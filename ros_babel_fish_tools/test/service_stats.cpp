// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

// End-to-end tests for the service_stats tool: the executable (path passed in via
// SERVICE_STATS_EXECUTABLE) is run against AddTwoInts servers created by this test and its output
// is checked.

#include "test_helpers.hpp"

#include <atomic>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <fstream>
#include <gtest/gtest.h>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <regex>

using namespace ros_babel_fish_tools_test;
using namespace std::chrono_literals;
using AddTwoInts = example_interfaces::srv::AddTwoInts;

namespace
{
const std::string kType = "example_interfaces/srv/AddTwoInts";
const std::string kService = "/rbf_tools_test/service_stats/add";
const std::string kSlowService = "/rbf_tools_test/service_stats/slow";
const std::string kMissingService = "/rbf_tools_test/service_stats/missing";

std::shared_ptr<rclcpp::Node> node;
rclcpp::Service<AddTwoInts>::SharedPtr service;
rclcpp::Service<AddTwoInts>::SharedPtr slow_service;

std::mutex requests_mutex;
std::vector<std::pair<int64_t, int64_t>> received_requests;
std::atomic<uint64_t> slow_requests = 0;

void clear_requests()
{
  std::lock_guard<std::mutex> lock( requests_mutex );
  received_requests.clear();
}

std::vector<std::pair<int64_t, int64_t>> requests()
{
  std::lock_guard<std::mutex> lock( requests_mutex );
  return received_requests;
}

// Output of a single call, e.g.
// "roundtrip 0.42 ms | request 20 B (serialized in 1.2 us) | response 12 B (deserialized in 0.8
// us)" Groups: 1 round trip ms, 2 request size, 3 response size
const std::regex kSingleCallLine( R"(^roundtrip ()" + kNum + R"() ms \| request ()" + kBytes +
                                  R"() \(serialized in )" + kNum + R"( us\) \| response ()" +
                                  kBytes + R"() \(deserialized in )" + kNum + R"( us\)$)" );
// Windowed report, e.g. "48 calls (48.0 Hz) | rtt ms min/avg/max 0.30/0.41/0.98 | deser us
// min/avg/max 0.5/0.7/1.2 | resp B min/avg/max 12/12/12 | bw 1.50 KiB/s[ | 3 timed out]"
// Groups: 1 call count, 2 rate, 3 rtt, 4 deser, 5 resp bytes (each min/avg/max or n/a),
// 6 timed out count (empty if absent)
const std::regex kReportLine( R"(^(\d+) calls \(()" + kNum + R"() Hz\) \| rtt ms min/avg/max ()" +
                              kMinAvgMax + R"(|n/a) \| deser us min/avg/max ()" + kMinAvgMax +
                              R"(|n/a) \| resp B min/avg/max ()" + kMinAvgMax + R"(|n/a) \| bw )" +
                              kBandwidth + R"((?: \| (\d+) timed out)?$)" );

std::optional<int> run_until_reports( ToolProcess &stats, size_t min_reports )
{
  return ros_babel_fish_tools_test::run_until_reports( stats, kReportLine, min_reports );
}

//! Reads the CSV at @p path and returns its header and data rows (split into fields).
void read_csv( const std::string &path, std::string &header,
               std::vector<std::vector<std::string>> &rows )
{
  std::ifstream csv( path );
  ASSERT_TRUE( csv.is_open() ) << path;
  ASSERT_TRUE( std::getline( csv, header ) );
  std::string line;
  while ( std::getline( csv, line ) ) rows.push_back( split_csv( line ) );
}
} // namespace

class ServiceStatsTool : public ::testing::Test
{
protected:
  void SetUp() override
  {
    clear_requests();
    slow_requests = 0;
  }
};

TEST_F( ServiceStatsTool, singleCallWithExplicitTypeAndRequest )
{
  TempDir dir;
  const std::string csv_path = dir.file( "calls.csv" );
  ToolProcess stats;
  ASSERT_TRUE( stats.start( SERVICE_STATS_EXECUTABLE,
                            { kService, kType, "--request", "{a: 40, b: 2}", "--out", csv_path } ) );
  const auto code = stats.wait( 15s );
  ASSERT_TRUE( code.has_value() ) << stats.stderr_text();
  EXPECT_EQ( *code, 0 ) << stats.stderr_text();
  // Not asserting an empty stderr: the middleware may log warnings (e.g. about shared memory in
  // containers) that are unrelated to the tool.
  EXPECT_EQ( stats.stderr_text().find( "Failed" ), std::string::npos ) << stats.stderr_text();

  const std::vector<std::string> lines = split_lines( stats.stdout_text() );
  ASSERT_EQ( lines.size(), 2u ) << stats.stdout_text();
  const std::regex header( R"(^Calling ')" + kService + "' \\[" + kType +
                           R"(\] with a custom request \(request )" + kBytes + R"(, serialized in )" +
                           kNum + R"( us\), writing measurements to ')" + csv_path + "' \\.\\.\\.$" );
  EXPECT_TRUE( std::regex_match( lines[0], header ) ) << lines[0];
  std::smatch m;
  ASSERT_TRUE( std::regex_match( lines[1], m, kSingleCallLine ) ) << lines[1];

  const size_t request_bytes = serialized_size( AddTwoInts::Request() );
  const size_t response_bytes = serialized_size( AddTwoInts::Response() );
  EXPECT_EQ( m[2].str(), std::to_string( request_bytes ) + " B" ) << lines[1];
  EXPECT_EQ( m[3].str(), std::to_string( response_bytes ) + " B" ) << lines[1];
  const double roundtrip_ms = std::stod( m[1] );
  EXPECT_GT( roundtrip_ms, 0.0 );
  EXPECT_LT( roundtrip_ms, 5000.0 );

  // Exactly one call with the requested payload reached the server.
  const auto reqs = requests();
  ASSERT_EQ( reqs.size(), 1u );
  EXPECT_EQ( reqs[0].first, 40 );
  EXPECT_EQ( reqs[0].second, 2 );

  std::string csv_header;
  std::vector<std::vector<std::string>> rows;
  read_csv( csv_path, csv_header, rows );
  EXPECT_EQ( csv_header, "call_ns,roundtrip_ns,success,response_bytes,deserialize_ns" );
  ASSERT_EQ( rows.size(), 1u );
  ASSERT_EQ( rows[0].size(), 5u );
  EXPECT_GT( std::stoll( rows[0][0] ), 0 );
  EXPECT_GT( std::stoll( rows[0][1] ), 0 );
  EXPECT_EQ( rows[0][2], "1" );
  EXPECT_EQ( std::stoull( rows[0][3] ), response_bytes );
  EXPECT_GT( std::stoll( rows[0][4] ), 0 );
  EXPECT_NEAR( std::stoll( rows[0][1] ) / 1e6, roundtrip_ms, 0.01 ) << "CSV and console agree";
}

TEST_F( ServiceStatsTool, singleCallWithDefaultRequestAndDetectedType )
{
  ToolProcess stats;
  ASSERT_TRUE( stats.start( SERVICE_STATS_EXECUTABLE, { kService } ) );
  const auto code = stats.wait( 15s );
  ASSERT_TRUE( code.has_value() ) << stats.stderr_text();
  EXPECT_EQ( *code, 0 ) << stats.stderr_text();

  const std::vector<std::string> lines = split_lines( stats.stdout_text() );
  ASSERT_EQ( lines.size(), 2u ) << stats.stdout_text();
  EXPECT_EQ( lines[0].rfind( "Calling '" + kService + "' [" + kType + "] (request ", 0 ), 0u )
      << lines[0];
  EXPECT_EQ( lines[0].find( "custom request" ), std::string::npos ) << lines[0];
  EXPECT_TRUE( std::regex_match( lines[1], kSingleCallLine ) ) << lines[1];

  const auto reqs = requests();
  ASSERT_EQ( reqs.size(), 1u );
  EXPECT_EQ( reqs[0].first, 0 ) << "default request is zero-initialized";
  EXPECT_EQ( reqs[0].second, 0 );
}

TEST_F( ServiceStatsTool, readsRequestFromFile )
{
  TempDir dir;
  const std::string request_path = dir.file( "request.yaml" );
  {
    std::ofstream f( request_path );
    f << "a: -5\nb: 12\n";
  }
  ToolProcess stats;
  ASSERT_TRUE( stats.start( SERVICE_STATS_EXECUTABLE,
                            { kService, kType, "--request-file", request_path } ) );
  const auto code = stats.wait( 15s );
  ASSERT_TRUE( code.has_value() ) << stats.stderr_text();
  EXPECT_EQ( *code, 0 ) << stats.stderr_text();
  EXPECT_NE( stats.stdout_text().find( "with a custom request" ), std::string::npos )
      << stats.stdout_text();

  const auto reqs = requests();
  ASSERT_EQ( reqs.size(), 1u );
  EXPECT_EQ( reqs[0].first, -5 );
  EXPECT_EQ( reqs[0].second, 12 );
}

TEST_F( ServiceStatsTool, singleCallTimeout )
{
  TempDir dir;
  const std::string csv_path = dir.file( "calls.csv" );
  ToolProcess stats;
  ASSERT_TRUE( stats.start( SERVICE_STATS_EXECUTABLE,
                            { kSlowService, kType, "--timeout", "0.3", "--out", csv_path } ) );
  const auto code = stats.wait( 15s );
  ASSERT_TRUE( code.has_value() ) << stats.stderr_text();
  EXPECT_EQ( *code, 1 ) << "a timed out single call is a failure";
  EXPECT_GE( slow_requests.load(), 1u ) << "the request must have reached the server";

  const std::vector<std::string> lines = split_lines( stats.stdout_text() );
  ASSERT_EQ( lines.size(), 2u ) << stats.stdout_text();
  EXPECT_EQ( lines[1], "timed out" );

  std::string csv_header;
  std::vector<std::vector<std::string>> rows;
  read_csv( csv_path, csv_header, rows );
  ASSERT_EQ( rows.size(), 1u );
  ASSERT_EQ( rows[0].size(), 5u ) << ::testing::PrintToString( rows[0] );
  EXPECT_GE( std::stoll( rows[0][1] ), 300'000'000 ) << "round trip covers the timeout";
  EXPECT_EQ( rows[0][2], "0" );
  EXPECT_TRUE( rows[0][3].empty() ) << "no response, no size";
  EXPECT_TRUE( rows[0][4].empty() ) << "no response, no deserialize time";
}

TEST_F( ServiceStatsTool, repeatedCallsReportPerWindow )
{
  TempDir dir;
  const std::string csv_path = dir.file( "calls.csv" );
  ToolProcess stats;
  ASSERT_TRUE( stats.start( SERVICE_STATS_EXECUTABLE, { kService, kType, "--rate", "50", "--window",
                                                        "1", "--out", csv_path } ) );
  const auto code = run_until_reports( stats, 2 );
  ASSERT_TRUE( code.has_value() ) << stats.stderr_text();
  EXPECT_EQ( *code, 0 ) << "Ctrl-C must lead to a clean exit. stderr:\n" << stats.stderr_text();
  EXPECT_EQ( stats.stderr_text().find( "Failed" ), std::string::npos ) << stats.stderr_text();

  const std::string out = stats.stdout_text();
  const std::vector<std::string> lines = split_lines( out );
  ASSERT_GE( lines.size(), 3u ) << out;
  EXPECT_EQ( lines[0].rfind( "Calling '" + kService + "' [" + kType +
                                 "], reporting every 1s at up to 50 Hz (request ",
                             0 ),
             0u )
      << lines[0];
  EXPECT_EQ( out.find( "roundtrip " ), std::string::npos ) << "no single-call line in rate mode";

  const std::vector<Match> reports = matching_lines( out, kReportLine );
  ASSERT_GE( reports.size(), 2u ) << out;
  const std::string response_bytes = std::to_string( serialized_size( AddTwoInts::Response() ) );
  uint64_t reported_calls = 0;
  for ( const Match &m : reports ) {
    const uint64_t count = std::stoull( m[1] );
    EXPECT_GT( count, 0u );
    reported_calls += count;
    const double hz = std::stod( m[2] );
    EXPECT_GT( hz, 10.0 ) << m[0];
    EXPECT_LE( hz, 60.0 ) << "must not exceed the requested rate: " << m[0];
    EXPECT_NE( m[3], "n/a" ) << m[0];
    EXPECT_NE( m[4], "n/a" ) << m[0];
    EXPECT_EQ( m[5], response_bytes + "/" + response_bytes + "/" + response_bytes ) << m[0];
    EXPECT_TRUE( m[6].empty() ) << "no timeouts expected: " << m[0];
  }
  EXPECT_LE( reported_calls, requests().size() );

  std::string csv_header;
  std::vector<std::vector<std::string>> rows;
  read_csv( csv_path, csv_header, rows );
  EXPECT_EQ( csv_header, "call_ns,roundtrip_ns,success,response_bytes,deserialize_ns" );
  EXPECT_GE( rows.size(), reported_calls );
  int64_t last_call = 0;
  for ( const auto &row : rows ) {
    ASSERT_EQ( row.size(), 5u ) << ::testing::PrintToString( row );
    const int64_t call_ns = std::stoll( row[0] );
    EXPECT_GE( call_ns, last_call );
    last_call = call_ns;
    EXPECT_EQ( row[2], "1" );
    EXPECT_EQ( row[3], response_bytes );
  }
}

TEST_F( ServiceStatsTool, repeatedCallsCountTimeouts )
{
  ToolProcess stats;
  ASSERT_TRUE( stats.start( SERVICE_STATS_EXECUTABLE, { kSlowService, kType, "--rate", "10",
                                                        "--window", "1", "--timeout", "0.1" } ) );
  const auto code = run_until_reports( stats, 1 );
  ASSERT_TRUE( code.has_value() ) << stats.stderr_text();
  EXPECT_EQ( *code, 0 );

  const std::vector<Match> reports = matching_lines( stats.stdout_text(), kReportLine );
  ASSERT_GE( reports.size(), 1u ) << stats.stdout_text();
  const Match &m = reports[0];
  ASSERT_FALSE( m[6].empty() ) << "timed out count expected: " << m[0];
  EXPECT_EQ( m[6], m[1] ) << "every call to the slow service times out: " << m[0];
  EXPECT_EQ( m[3], "n/a" ) << "no successful round trip: " << m[0];
  EXPECT_EQ( m[5], "n/a" ) << m[0];
  EXPECT_NE( m[0].find( "| bw " ), std::string::npos )
      << "requests were still sent, so bandwidth is reported: " << m[0];
  EXPECT_GE( slow_requests.load(), std::stoull( m[1] ) ) << "every attempt reached the server";
}

TEST_F( ServiceStatsTool, waitsForMissingServiceUntilInterrupted )
{
  ToolProcess stats;
  ASSERT_TRUE( stats.start( SERVICE_STATS_EXECUTABLE, { kMissingService, kType } ) );
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while ( std::chrono::steady_clock::now() < deadline && stats.running() &&
          stats.stderr_text().find( "Waiting for service" ) == std::string::npos )
    std::this_thread::sleep_for( 50ms );
  EXPECT_NE( stats.stderr_text().find( "Waiting for service '" + kMissingService +
                                       "' to become available..." ),
             std::string::npos )
      << stats.stderr_text();
  EXPECT_TRUE( stats.running() );
  stats.interrupt();
  const auto code = stats.wait( 10s );
  ASSERT_TRUE( code.has_value() );
  EXPECT_EQ( *code, 0 );
  EXPECT_TRUE( stats.stdout_text().empty() ) << "no call was made: " << stats.stdout_text();
}

TEST_F( ServiceStatsTool, failsIfTypeCannotBeDetected )
{
  ToolProcess stats;
  ASSERT_TRUE( stats.start( SERVICE_STATS_EXECUTABLE, { kMissingService } ) );
  const auto code = stats.wait( 20s );
  ASSERT_TRUE( code.has_value() ) << "type detection must give up after its timeout";
  EXPECT_EQ( *code, 1 );
  EXPECT_NE( stats.stderr_text().find( "Could not determine the type of service '" +
                                       kMissingService + "'" ),
             std::string::npos )
      << stats.stderr_text();
}

TEST_F( ServiceStatsTool, helpExitsSuccessfully )
{
  expect_help_on_stderr( SERVICE_STATS_EXECUTABLE, "--request-file" );
}

TEST_F( ServiceStatsTool, rejectsInvalidArguments )
{
  TempDir dir;
  const std::vector<InvalidArgumentCase> cases = {
      { {}, "Usage:" },
      { { "--rate", "1" }, "Usage:" },
      { { kService, kType, "--bogus" }, "Unknown option: --bogus" },
      { { kService, kType, "extra" }, "Too many positional arguments: extra" },
      { { kService, kType, "--rate" }, "Missing value for --rate" },
      { { kService, kType, "--rate", "0" }, "--rate must be a positive number of calls per second" },
      { { kService, kType, "--rate", "x" }, "--rate must be a positive number of calls per second" },
      { { kService, kType, "--window", "0" }, "--window must be a positive number of seconds" },
      { { kService, kType, "--timeout", "-1" }, "--timeout must be a positive number of seconds" },
      { { kService, kType, "--timeout" }, "Missing value for --timeout" },
      { { kService, kType, "--request" }, "Missing value for --request" },
      { { kService, kType, "--request", "{a: 1}", "--request-file", "x.yaml" },
        "--request and --request-file are mutually exclusive" },
      { { kService, kType, "--request-file", dir.file( "missing.yaml" ) },
        "Failed to open request file" },
      { { kService, kType, "--request", "[1, 2]" }, "Failed to set up service client" },
      { { kService, "no_such_pkg/srv/Nope" }, "Failed to set up service client" },
      { { kService, kType, "--out", dir.file( "missing_dir/out.csv" ) },
        "Failed to open output file" },
  };
  expect_rejects_arguments( SERVICE_STATS_EXECUTABLE, cases );
  EXPECT_TRUE( requests().empty() ) << "argument errors must be caught before any call";
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  node = std::make_shared<rclcpp::Node>( "service_stats_tool_test" );
  service = node->create_service<AddTwoInts>( kService, []( const AddTwoInts::Request::SharedPtr req,
                                                            AddTwoInts::Response::SharedPtr resp ) {
    std::lock_guard<std::mutex> lock( requests_mutex );
    received_requests.emplace_back( req->a, req->b );
    resp->sum = req->a + req->b;
  } );
  // The "slow" service takes the deferred-response form of the callback and never answers, so
  // every call to it times out without blocking the executor.
  slow_service = node->create_service<AddTwoInts>(
      kSlowService, []( const std::shared_ptr<rmw_request_id_t>,
                        const AddTwoInts::Request::SharedPtr ) { ++slow_requests; } );
  std::thread spinner( []() { rclcpp::spin( node ); } );

  const int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  spinner.join();
  service.reset();
  slow_service.reset();
  node.reset();
  return result;
}
