// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "stats_common.hpp"

#include <ros_babel_fish/babel_fish.hpp>
#include <ros_babel_fish_tools/cli.hpp>
#include <ros_babel_fish_tools/yaml_cpp_serialization.hpp>

#include <chrono>
#include <format>
#include <fstream>
#include <iostream>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace ros_babel_fish;
using namespace ros_babel_fish_tools;

void print_usage( const char *name )
{
  std::cerr << std::format(
      R"(Usage: {0} <service> [type] [options]
Call a service and report round-trip latency. By default a single call is made;
pass --rate to call repeatedly and also report the achieved call rate.
If [type] is omitted it is auto-detected from the ROS graph.
Examples:
  # Single call, auto-detecting the service type:
  {0} /my_service
  # Single call with an explicit type and request payload:
  {0} /add_two_ints example_interfaces/srv/AddTwoInts --request '{{a: 1, b: 2}}'
  # Call repeatedly at up to 10 Hz, reporting every 2 seconds:
  {0} /my_service --rate 10 --window 2
  # Pass ROS arguments; '--' ends the ROS args so --rate is parsed by this tool:
  {0} /my_service --ros-args -p use_sim_time:=true -- --rate 10
Options:
  -h, --help            Show this help message
  --rate <hz>           Call repeatedly at up to this many calls per second
                        (default: perform a single call and exit)
  --window <seconds>    Reporting interval in seconds; only used with --rate
                        (default: 5)
  --timeout <seconds>   Per-call response timeout; a call that exceeds it is counted
                        as timed out (default: wait indefinitely)
  --request <yaml>      Request payload as an inline YAML map (default: empty request)
  --request-file <file> Read the request payload from a YAML file
  --out <file>          Write per-call measurements to a CSV file
  --ros-args ...        Pass ROS arguments (e.g. -p use_sim_time:=true).
)",
      name );
}

//! All metrics collected within one reporting window.
struct WindowStats {
  Accumulator roundtrip_ns; //!< request sent -> response received
  uint64_t call_count = 0;  //!< completed attempts (successes + timeouts)
  uint64_t timeout_count = 0;

  void reset() { *this = WindowStats{}; }
};

//! Looks up the type of @p service in the ROS graph, waiting up to @p timeout_s for it to appear.
//! @return The first advertised type, or an empty string if none was found before the timeout.
std::string detect_service_type( rclcpp::Node &node, rclcpp::Executor &executor,
                                 const std::string &service, double timeout_s )
{
  std::string fq_name;
  try {
    fq_name = node.get_node_base_interface()->resolve_topic_or_service_name( service, true );
  } catch ( const std::exception & ) {
    fq_name = service;
  }
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::nanoseconds( static_cast<int64_t>( timeout_s * 1e9 ) );
  while ( rclcpp::ok() ) {
    const std::map<std::string, std::vector<std::string>> names_and_types =
        node.get_service_names_and_types();
    const auto it = names_and_types.find( fq_name );
    if ( it != names_and_types.end() && !it->second.empty() )
      return it->second.front();
    if ( std::chrono::steady_clock::now() >= deadline )
      break;
    // The graph cache is populated by the middleware; spin briefly so updates are processed.
    executor.spin_some();
    std::this_thread::sleep_for( std::chrono::milliseconds( 50 ) );
  }
  return {};
}

int main( int argc, char **argv )
{
  // Turn off Zenoh logging to keep the periodic stats output clean.
  ros_babel_fish_tools::silence_rmw_logging();
  // Strip ROS arguments (e.g. --ros-args -p use_sim_time:=true) before our own parsing; the node
  // picks them up automatically via the global context.
  const std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments( argc, argv );

  std::string service;
  std::string type;
  std::string out_path;
  std::string request_yaml;
  std::string request_file;
  double window = 5.0;
  double rate = 0.0;    // 0 => call as fast as possible
  double timeout = 0.0; // 0 => wait indefinitely for each response
  for ( size_t i = 1; i < args.size(); ++i ) {
    const std::string &arg = args[i];
    if ( arg == "-h" || arg == "--help" ) {
      print_usage( argv[0] );
      return 0;
    }
    std::string value;
    if ( arg == "--window" || arg == "--timeout" ) {
      if ( !take_option_value( args, i, value ) ) {
        print_usage( argv[0] );
        return 1;
      }
      if ( !parse_positive_seconds( value, arg == "--window" ? window : timeout ) ) {
        std::cerr << arg << " must be a positive number of seconds" << std::endl;
        return 1;
      }
    } else if ( arg == "--rate" ) {
      if ( !take_option_value( args, i, value ) ) {
        print_usage( argv[0] );
        return 1;
      }
      if ( !parse_positive_number( value, rate ) ) {
        std::cerr << "--rate must be a positive number of calls per second" << std::endl;
        return 1;
      }
    } else if ( arg == "--request" || arg == "--request-file" || arg == "--out" ) {
      std::string &target = arg == "--request"        ? request_yaml
                            : arg == "--request-file" ? request_file
                                                      : out_path;
      if ( !take_option_value( args, i, target ) ) {
        print_usage( argv[0] );
        return 1;
      }
    } else if ( !arg.empty() && arg[0] == '-' ) {
      std::cerr << "Unknown option: " << arg << std::endl;
      print_usage( argv[0] );
      return 1;
    } else if ( service.empty() ) {
      service = arg;
    } else if ( type.empty() ) {
      type = arg;
    } else {
      std::cerr << "Too many positional arguments: " << arg << std::endl;
      print_usage( argv[0] );
      return 1;
    }
  }

  if ( service.empty() ) {
    print_usage( argv[0] );
    return 1;
  }

  if ( !request_yaml.empty() && !request_file.empty() ) {
    std::cerr << "--request and --request-file are mutually exclusive" << std::endl;
    return 1;
  }
  if ( !request_file.empty() ) {
    std::ifstream in( request_file );
    if ( !in.is_open() ) {
      std::cerr << "Failed to open request file: " << request_file << std::endl;
      return 1;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    request_yaml = buffer.str();
  }

  std::ofstream csv;
  if ( !out_path.empty() ) {
    if ( !open_csv_output( out_path, csv ) )
      return 1;
    csv << "call_ns,roundtrip_ns,success\n";
  }

  auto node = std::make_shared<rclcpp::Node>( "ros_babel_fish_service_stats" );
  BabelFish fish;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node( node );

  if ( type.empty() ) {
    type = detect_service_type( *node, executor, service, 5.0 );
    if ( type.empty() ) {
      std::cerr << "Could not determine the type of service '" << service
                << "'; specify it as the second argument." << std::endl;
      return 1;
    }
  }

  BabelFishServiceClient::SharedPtr client;
  CompoundMessage::SharedPtr request;
  try {
    client = fish.create_service_client( *node, service, type );
    request = fish.create_service_request_shared( type );
    if ( !request_yaml.empty() )
      yaml_to_message( YAML::Load( request_yaml ), *request );
  } catch ( const std::exception &e ) {
    std::cerr << "Failed to set up service client: " << e.what() << std::endl;
    return 1;
  }

  if ( !client ) {
    std::cerr << "Could not create service client!" << std::endl;
    return 1;
  }

  // Wait for the service to come up so the first measured call is not skewed by discovery.
  while ( rclcpp::ok() && !client->wait_for_service( std::chrono::seconds( 1 ) ) ) {
    std::cerr << "Waiting for service '" << service << "' to become available..." << std::endl;
  }
  if ( !rclcpp::ok() )
    return 0;

  // Without --rate the tool performs a single call; --rate switches to repeated, windowed reporting.
  const bool repeat = rate > 0.0;

  std::cout << "Calling '" << service << "' [" << type << "]";
  if ( repeat )
    std::cout << ", reporting every " << window << "s at up to " << rate << " Hz";
  if ( !request_yaml.empty() )
    std::cout << " with a custom request";
  if ( csv.is_open() )
    std::cout << ", writing measurements to '" << out_path << "'";
  std::cout << " ..." << std::endl;

  WindowStats stats;
  const auto timeout_dur = std::chrono::nanoseconds( static_cast<int64_t>( timeout * 1e9 ) );
  const auto call_period = repeat ? std::chrono::nanoseconds( static_cast<int64_t>( 1e9 / rate ) )
                                  : std::chrono::nanoseconds( 0 );

  auto last_report = std::chrono::steady_clock::now();
  auto report = [&stats, &last_report]() {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>( now - last_report ).count();
    last_report = now;

    if ( stats.call_count == 0 ) {
      std::cout << "no calls completed" << std::endl;
      stats.reset();
      return;
    }

    std::string line =
        std::format( "{} calls ({:.1f} Hz) | rtt ms min/avg/max {}", stats.call_count,
                     stats.call_count / elapsed, format_min_avg_max( stats.roundtrip_ns, 1e6, 2 ) );
    if ( stats.timeout_count != 0 )
      line += std::format( " | {} timed out", stats.timeout_count );
    std::cout << line << std::endl;

    stats.reset();
  };

  auto next_call = std::chrono::steady_clock::now();
  while ( rclcpp::ok() ) {
    if ( repeat ) {
      std::this_thread::sleep_until( next_call );
      next_call += call_period;
      // If the service is slower than the requested rate, don't build up a backlog of calls.
      const auto now = std::chrono::steady_clock::now();
      if ( next_call < now )
        next_call = now;
    }

    const rclcpp::Time call_time = node->now();
    const auto start = std::chrono::steady_clock::now();
    auto future = client->async_send_request( request );
    const rclcpp::FutureReturnCode ret =
        timeout > 0.0 ? executor.spin_until_future_complete( future.future, timeout_dur )
                      : executor.spin_until_future_complete( future.future );
    const auto end = std::chrono::steady_clock::now();

    if ( ret == rclcpp::FutureReturnCode::INTERRUPTED )
      break; // rclcpp is shutting down (e.g. Ctrl-C)

    const int64_t roundtrip_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>( end - start ).count();
    const bool success = ret == rclcpp::FutureReturnCode::SUCCESS;
    if ( success ) {
      stats.roundtrip_ns.add( roundtrip_ns );
    } else {
      // Drop the still-pending request so it doesn't accumulate in the client.
      client->remove_pending_request( future );
      ++stats.timeout_count;
    }
    ++stats.call_count;

    if ( csv.is_open() )
      csv << call_time.nanoseconds() << ',' << roundtrip_ns << ',' << ( success ? 1 : 0 ) << '\n';

    if ( !repeat ) {
      // Single-call mode: report this one measurement and exit (non-zero on a timed-out call).
      if ( csv.is_open() )
        csv.flush();
      if ( success ) {
        std::cout << std::format( "roundtrip: {:.2f} ms", roundtrip_ns / 1e6 ) << std::endl;
      } else {
        std::cout << "timed out" << std::endl;
      }
      return success ? 0 : 1;
    }

    if ( std::chrono::duration<double>( end - last_report ).count() >= window ) {
      if ( csv.is_open() )
        csv.flush();
      report();
    }
  }

  return 0;
}
