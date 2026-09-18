// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

// End-to-end tests for the topic_stats tool: the executable (path passed in via
// TOPIC_STATS_EXECUTABLE) is run against a publisher created by this test and its periodic report
// lines and CSV output are checked.

#include "test_helpers.hpp"

#include <atomic>
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <std_msgs/msg/int32.hpp>

using namespace ros_babel_fish_tools_test;
using namespace std::chrono_literals;

namespace
{
std::shared_ptr<rclcpp::Node> node;
const std::string kTopicPrefix = "/rbf_tools_test/topic_stats/";
const std::string kPoseType = "geometry_msgs/msg/PoseStamped";

// Groups: 1 message count, 2 rate, 3 latency min/avg/max or n/a
const std::regex kReportLine( R"(^(\d+) msgs \(()" + kNum + R"() Hz\) \| latency ms min/avg/max ()" +
                              kMinAvgMax + R"(|n/a) \| deser us min/avg/max )" + kMinAvgMax +
                              R"( \| bw )" + kBandwidth + "$" );
// Groups: 1 algorithm
const std::regex kCompressionLine( R"(^  (lz4|zstd) bw )" + kBandwidth + R"( \()" + kNum +
                                   R"(x\) \| compress us min/avg/max )" + kMinAvgMax +
                                   R"( \| decompress us min/avg/max )" + kMinAvgMax + "$" );

//! Publishes @p make_msg() at 50 Hz on @p topic on a background thread until destroyed.
template<typename MessageT>
class Publisher
{
public:
  Publisher( const std::string &topic, std::function<MessageT()> make_msg )
      : pub_( node->create_publisher<MessageT>( topic, 10 ) ),
        thread_( [this, make_msg = std::move( make_msg )]() {
          while ( !stop_ ) {
            pub_->publish( make_msg() );
            ++published_;
            std::this_thread::sleep_for( 20ms );
          }
        } )
  {
  }

  ~Publisher()
  {
    stop_ = true;
    thread_.join();
  }

  uint64_t published() const { return published_; }

private:
  typename rclcpp::Publisher<MessageT>::SharedPtr pub_;
  std::atomic<bool> stop_ = false;
  std::atomic<uint64_t> published_ = 0;
  std::thread thread_;
};

geometry_msgs::msg::PoseStamped make_stamped_pose()
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = node->now();
  msg.pose.orientation.w = 1.0;
  return msg;
}

std::optional<int> run_until_reports( ToolProcess &stats, size_t min_reports )
{
  return ros_babel_fish_tools_test::run_until_reports( stats, kReportLine, min_reports );
}
} // namespace

TEST( TopicStatsTool, reportsLatencyDeserializationAndBandwidth )
{
  const std::string topic = kTopicPrefix + "stamped";
  TempDir dir;
  const std::string csv_path = dir.file( "stats.csv" );
  Publisher<geometry_msgs::msg::PoseStamped> pub( topic, make_stamped_pose );

  ToolProcess stats;
  ASSERT_TRUE( stats.start( TOPIC_STATS_EXECUTABLE,
                            { topic, kPoseType, "--window", "1", "--out", csv_path } ) );
  const auto code = run_until_reports( stats, 2 );
  ASSERT_TRUE( code.has_value() ) << stats.stderr_text();
  EXPECT_EQ( *code, 0 ) << "Ctrl-C must lead to a clean exit. stderr:\n" << stats.stderr_text();

  const std::string out = stats.stdout_text();
  const std::vector<std::string> lines = split_lines( out );
  ASSERT_GE( lines.size(), 3u ) << out;
  EXPECT_EQ( lines[0], "Monitoring '" + topic + "' [" + kPoseType +
                           "], reporting every 1s ... writing measurements to '" + csv_path + "'" );

  const std::vector<Match> reports = matching_lines( out, kReportLine );
  ASSERT_GE( reports.size(), 2u ) << out;
  uint64_t reported_messages = 0;
  for ( const Match &m : reports ) {
    const uint64_t count = std::stoull( m[1] );
    EXPECT_GT( count, 0u );
    reported_messages += count;
    // ~50 Hz publisher; allow generous slack for scheduling and the partial first window.
    const double hz = std::stod( m[2] );
    EXPECT_GT( hz, 10.0 ) << m[0];
    EXPECT_LT( hz, 120.0 ) << m[0];
    EXPECT_NE( m[3], "n/a" ) << "stamped messages must yield a latency: " << m[0];
  }
  EXPECT_LE( reported_messages, pub.published() );
  EXPECT_EQ( out.find( "no messages" ), std::string::npos ) << out;
  EXPECT_EQ( stats.stderr_text().find( "header" ), std::string::npos )
      << "no latency warning expected for stamped messages: " << stats.stderr_text();

  // Per-message CSV: header + one row per received message.
  std::ifstream csv( csv_path );
  ASSERT_TRUE( csv.is_open() );
  std::string line;
  ASSERT_TRUE( std::getline( csv, line ) );
  EXPECT_EQ( line, "recv_ns,latency_ns,deserialize_ns,size_bytes" );
  const size_t expected_size = serialized_size( make_stamped_pose() );
  size_t rows = 0;
  int64_t last_recv = 0;
  while ( std::getline( csv, line ) ) {
    const std::vector<std::string> fields = split_csv( line );
    ASSERT_EQ( fields.size(), 4u ) << line;
    const int64_t recv = std::stoll( fields[0] );
    EXPECT_GE( recv, last_recv ) << "receive times must be monotonic: " << line;
    last_recv = recv;
    EXPECT_FALSE( fields[1].empty() ) << "latency must be present: " << line;
    EXPECT_NO_THROW( std::stoll( fields[1] ) ) << line;
    EXPECT_GT( std::stoll( fields[2] ), 0 ) << "deserialize time: " << line;
    EXPECT_EQ( std::stoull( fields[3] ), expected_size ) << line;
    ++rows;
  }
  EXPECT_GE( rows, reported_messages ) << "every reported message has a CSV row";
  EXPECT_LE( rows, pub.published() );
}

TEST( TopicStatsTool, reportsNotAvailableLatencyWithoutHeader )
{
  const std::string topic = kTopicPrefix + "no_header";
  Publisher<std_msgs::msg::Int32> pub( topic, []() {
    std_msgs::msg::Int32 msg;
    msg.data = 1;
    return msg;
  } );

  ToolProcess stats;
  ASSERT_TRUE(
      stats.start( TOPIC_STATS_EXECUTABLE, { topic, "std_msgs/msg/Int32", "--window", "1" } ) );
  const auto code = run_until_reports( stats, 1 );
  ASSERT_TRUE( code.has_value() ) << stats.stderr_text();
  EXPECT_EQ( *code, 0 );

  const std::vector<Match> reports = matching_lines( stats.stdout_text(), kReportLine );
  ASSERT_GE( reports.size(), 1u ) << stats.stdout_text();
  for ( const Match &m : reports ) EXPECT_EQ( m[3], "n/a" ) << m[0];

  const std::string err = stats.stderr_text();
  EXPECT_NE( err.find( "Topic '" + topic +
                       "' message type 'std_msgs/msg/Int32' has no usable "
                       "'header' stamp; latency will not be measured." ),
             std::string::npos )
      << err;
  // Warned exactly once, not per message.
  EXPECT_EQ( err.find( "has no usable" ), err.rfind( "has no usable" ) ) << err;
}

TEST( TopicStatsTool, detectsTypeFromGraph )
{
  const std::string topic = kTopicPrefix + "autodetect";
  Publisher<geometry_msgs::msg::PoseStamped> pub( topic, make_stamped_pose );

  ToolProcess stats;
  ASSERT_TRUE( stats.start( TOPIC_STATS_EXECUTABLE, { topic, "--window", "1" } ) );
  const auto code = run_until_reports( stats, 1 );
  ASSERT_TRUE( code.has_value() ) << stats.stderr_text();
  EXPECT_EQ( *code, 0 );
  const std::vector<std::string> lines = split_lines( stats.stdout_text() );
  ASSERT_FALSE( lines.empty() );
  EXPECT_EQ( lines[0], "Monitoring '" + topic + "' [" + kPoseType + "], reporting every 1s ..." );
  EXPECT_GE( matching_lines( stats.stdout_text(), kReportLine ).size(), 1u ) << stats.stdout_text();
}

TEST( TopicStatsTool, reportsNoMessagesWhenIdle )
{
  const std::string topic = kTopicPrefix + "idle";
  ToolProcess stats;
  ASSERT_TRUE( stats.start( TOPIC_STATS_EXECUTABLE, { topic, kPoseType, "--window", "0.5" } ) );
  ASSERT_TRUE( stats.wait_for_stdout( "no messages", 10s ) ) << stats.stdout_text();
  stats.interrupt();
  const auto code = stats.wait( 10s );
  ASSERT_TRUE( code.has_value() );
  EXPECT_EQ( *code, 0 );
  EXPECT_EQ( matching_lines( stats.stdout_text(), kReportLine ).size(), 0u ) << stats.stdout_text();
}

class TopicStatsCompression : public ::testing::TestWithParam<std::string>
{
};

TEST_P( TopicStatsCompression, reportsCompressedBandwidth )
{
  const std::string algo = GetParam();
  const std::string topic = kTopicPrefix + "compress_" + algo;
  TempDir dir;
  const std::string csv_path = dir.file( "stats.csv" );
  Publisher<geometry_msgs::msg::PoseStamped> pub( topic, make_stamped_pose );

  ToolProcess stats;
  ASSERT_TRUE( stats.start( TOPIC_STATS_EXECUTABLE, { topic, kPoseType, "--window", "1",
                                                      "--compress", algo, "--out", csv_path } ) );
  const auto code = run_until_reports( stats, 2 );
  ASSERT_TRUE( code.has_value() ) << stats.stderr_text();
  EXPECT_EQ( *code, 0 );

  const std::string out = stats.stdout_text();
  const std::vector<std::string> lines = split_lines( out );
  ASSERT_FALSE( lines.empty() );
  EXPECT_NE( lines[0].find( "compressing with " + algo ), std::string::npos ) << lines[0];

  const std::vector<Match> reports = matching_lines( out, kReportLine );
  const std::vector<Match> compression = matching_lines( out, kCompressionLine );
  ASSERT_GE( reports.size(), 2u ) << out;
  EXPECT_EQ( compression.size(), reports.size() ) << "one compression line per report line:\n"
                                                  << out;
  for ( const Match &m : compression ) EXPECT_EQ( m[1], algo ) << m[0];
  // The compression line directly follows its report line.
  for ( size_t i = 0; i + 1 < lines.size(); ++i ) {
    if ( std::regex_match( lines[i], kReportLine ) ) {
      EXPECT_TRUE( std::regex_match( lines[i + 1], kCompressionLine ) ) << lines[i + 1];
    }
  }
  EXPECT_EQ( stats.stderr_text().find( "Compression failed" ), std::string::npos )
      << stats.stderr_text();

  std::ifstream csv( csv_path );
  ASSERT_TRUE( csv.is_open() );
  std::string line;
  ASSERT_TRUE( std::getline( csv, line ) );
  EXPECT_EQ( line, "recv_ns,latency_ns,deserialize_ns,size_bytes,compress_ns,decompress_ns,"
                   "compressed_size_bytes" );
  size_t rows = 0;
  while ( std::getline( csv, line ) ) {
    const std::vector<std::string> fields = split_csv( line );
    ASSERT_EQ( fields.size(), 7u ) << line;
    EXPECT_GT( std::stoll( fields[4] ), 0 ) << "compress time: " << line;
    EXPECT_GT( std::stoll( fields[5] ), 0 ) << "decompress time: " << line;
    EXPECT_GT( std::stoull( fields[6] ), 0u ) << "compressed size: " << line;
    ++rows;
  }
  EXPECT_GT( rows, 0u );
}

INSTANTIATE_TEST_SUITE_P( Algorithms, TopicStatsCompression, ::testing::Values( "lz4", "zstd" ),
                          []( const ::testing::TestParamInfo<std::string> &info ) {
                            return info.param;
                          } );

TEST( TopicStatsTool, helpExitsSuccessfully )
{
  expect_help_on_stderr( TOPIC_STATS_EXECUTABLE, "--compress" );
}

TEST( TopicStatsTool, rejectsInvalidArguments )
{
  const std::string topic = kTopicPrefix + "invalid";
  TempDir dir;
  const std::vector<InvalidArgumentCase> cases = {
      { {}, "Usage:" },
      { { "--window", "1" }, "Usage:" },
      { { topic, kPoseType, "--bogus" }, "Unknown option: --bogus" },
      { { topic, kPoseType, "extra" }, "Too many positional arguments: extra" },
      { { topic, kPoseType, "--window" }, "Missing value for --window" },
      { { topic, kPoseType, "--window", "0" }, "--window must be a positive number of seconds" },
      { { topic, kPoseType, "--window", "-1" }, "--window must be a positive number of seconds" },
      { { topic, kPoseType, "--window", "abc" }, "--window must be a positive number of seconds" },
      { { topic, kPoseType, "--out" }, "Missing value for --out" },
      { { topic, kPoseType, "--compress" }, "Missing value for --compress" },
      { { topic, kPoseType, "--compress", "gzip" }, "--compress must be 'lz4' or 'zstd', got: gzip" },
      { { topic, "no_such_pkg/msg/Nope" }, "Failed to create subscription" },
      { { topic, kPoseType, "--out", dir.file( "missing_dir/out.csv" ) },
        "Failed to open output file" },
  };
  expect_rejects_arguments( TOPIC_STATS_EXECUTABLE, cases );
}

TEST( TopicStatsTool, doesNotOverwriteOutputFileWithoutConfirmation )
{
  // The prompt itself is covered by the stats_common unit tests; this checks the tool wires it up
  // and that the answer is read from stdin.
  const std::string topic = kTopicPrefix + "overwrite";
  TempDir dir;
  const std::string csv_path = dir.file( "existing.csv" );
  {
    std::ofstream f( csv_path );
    f << "keep me\n";
  }

  ToolProcess stats;
  ASSERT_TRUE( stats.start( TOPIC_STATS_EXECUTABLE, { topic, kPoseType, "--out", csv_path }, "n\n" ) );
  const auto code = stats.wait( 10s );
  ASSERT_TRUE( code.has_value() );
  EXPECT_EQ( *code, 1 );
  EXPECT_NE( stats.stdout_text().find( "already exists. Overwrite? [y/N]" ), std::string::npos )
      << stats.stdout_text();
  EXPECT_NE( stats.stderr_text().find( "Aborting; output file not overwritten." ), std::string::npos )
      << stats.stderr_text();
  std::ifstream f( csv_path );
  std::string content;
  std::getline( f, content );
  EXPECT_EQ( content, "keep me" );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  node = std::make_shared<rclcpp::Node>( "topic_stats_tool_test" );
  const int result = RUN_ALL_TESTS();
  node.reset();
  rclcpp::shutdown();
  return result;
}
