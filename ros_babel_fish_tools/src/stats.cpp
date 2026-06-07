// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <ros_babel_fish/babel_fish.hpp>
#include <ros_babel_fish_tools/cli.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

using namespace ros_babel_fish;

void print_usage( const char *name )
{
  std::cerr << "Usage: " << name << " <topic> [type] [options]" << std::endl;
  std::cerr << "Subscribe to a topic and report latency, bandwidth and deserialize time."
            << std::endl;
  std::cerr << "Latency requires the message to have a std_msgs/Header 'header' field." << std::endl;
  std::cerr << "Options:" << std::endl;
  std::cerr << "  -h, --help          Show this help message" << std::endl;
  std::cerr << "  --window <seconds>  Reporting interval in seconds (default: 5)" << std::endl;
  std::cerr << "  --out <file>        Write per-message measurements to a CSV file" << std::endl;
}

//! Tracks min/max/sum/count of a single metric over a reporting window.
struct Accumulator {
  uint64_t count = 0;
  int64_t sum = 0;
  int64_t min = 0;
  int64_t max = 0;

  void add( int64_t value )
  {
    if ( count == 0 ) {
      min = max = value;
    } else {
      min = std::min( min, value );
      max = std::max( max, value );
    }
    sum += value;
    ++count;
  }

  double avg() const { return count == 0 ? 0.0 : static_cast<double>( sum ) / count; }

  void reset() { *this = Accumulator{}; }
};

//! All metrics collected within one reporting window.
struct WindowStats {
  Accumulator latency_ns;     //!< header stamp -> receive time
  Accumulator deserialize_ns; //!< time spent deserializing the message
  uint64_t message_count = 0;
  uint64_t total_bytes = 0;

  void reset() { *this = WindowStats{}; }
};

std::string format_bytes_per_sec( double bytes_per_sec )
{
  const char *units[] = { "B/s", "KiB/s", "MiB/s", "GiB/s" };
  int unit = 0;
  while ( bytes_per_sec >= 1024.0 && unit < 3 ) {
    bytes_per_sec /= 1024.0;
    ++unit;
  }
  std::ostringstream out;
  out << std::fixed << std::setprecision( unit == 0 ? 0 : 2 ) << bytes_per_sec << " " << units[unit];
  return out.str();
}

//! Reads the header stamp of a message as an rclcpp::Time.
//! @return False if the message has no header or no time stamp field.
bool try_get_stamp( const CompoundMessage &msg, rclcpp::Time &stamp_out )
{
  if ( !msg.containsKey( "header" ) )
    return false;
  try {
    const Message &stamp = msg["header"]["stamp"];
    if ( !stamp.isTime() )
      return false;
    stamp_out = stamp.value<rclcpp::Time>();
    return true;
  } catch ( const std::exception & ) {
    return false;
  }
}

int main( int argc, char **argv )
{
  // Turn off Zenoh logging to keep the periodic stats output clean.
  ros_babel_fish_tools::silence_rmw_logging();
  rclcpp::init( argc, argv );

  std::string topic;
  std::string type;
  std::string out_path;
  double window = 5.0;
  for ( int i = 1; i < argc; ++i ) {
    std::string arg = argv[i];
    if ( arg == "-h" || arg == "--help" ) {
      print_usage( argv[0] );
      return 0;
    }
    if ( arg == "--window" ) {
      if ( i + 1 >= argc ) {
        std::cerr << "Missing value for --window" << std::endl;
        print_usage( argv[0] );
        return 1;
      }
      const char *value = argv[++i];
      char *end = nullptr;
      window = std::strtod( value, &end );
      // Reject junk ("5x"), empty parses and values large enough to overflow the timer period.
      if ( end == value || *end != '\0' || !std::isfinite( window ) || window <= 0.0 ||
           window > static_cast<double>( std::numeric_limits<int64_t>::max() ) / 1e9 ) {
        std::cerr << "--window must be a positive number of seconds" << std::endl;
        return 1;
      }
    } else if ( arg == "--out" ) {
      if ( i + 1 >= argc ) {
        std::cerr << "Missing value for --out" << std::endl;
        print_usage( argv[0] );
        return 1;
      }
      out_path = argv[++i];
    } else if ( !arg.empty() && arg[0] == '-' ) {
      std::cerr << "Unknown option: " << arg << std::endl;
      print_usage( argv[0] );
      return 1;
    } else if ( topic.empty() ) {
      topic = arg;
    } else if ( type.empty() ) {
      type = arg;
    } else {
      std::cerr << "Too many positional arguments: " << arg << std::endl;
      print_usage( argv[0] );
      return 1;
    }
  }

  if ( topic.empty() ) {
    print_usage( argv[0] );
    return 1;
  }

  std::ofstream csv;
  if ( !out_path.empty() ) {
    if ( std::filesystem::exists( out_path ) ) {
      std::cout << "Output file '" << out_path
                << "' already exists. Overwrite? [y/N]: " << std::flush;
      std::string answer;
      // A non-interactive stdin (EOF) leaves answer empty, i.e. defaults to not overwriting.
      std::getline( std::cin, answer );
      if ( answer != "y" && answer != "Y" && answer != "yes" ) {
        std::cerr << "Aborting; output file not overwritten." << std::endl;
        return 1;
      }
    }
    csv.open( out_path );
    if ( !csv.is_open() ) {
      std::cerr << "Failed to open output file: " << out_path << std::endl;
      return 1;
    }
    csv << "recv_ns,latency_ns,deserialize_ns,size_bytes\n";
  }

  auto node = std::make_shared<rclcpp::Node>( "ros_babel_fish_stats" );
  BabelFish fish;

  WindowStats stats;
  bool latency_warned = false;
  BabelFishSubscription::SharedPtr sub;

  // A serialized-message callback makes BabelFish deliver the raw bytes, which gives us the
  // message size for bandwidth and lets us time the deserialization ourselves.
  auto callback = [&sub, &stats, &latency_warned, &csv, node = node.get(),
                   topic]( std::shared_ptr<rclcpp::SerializedMessage> serialized ) {
    const rclcpp::Time recv = node->now();
    const size_t size = serialized->size();
    stats.total_bytes += size;
    stats.message_count += 1;

    CompoundMessage compound;
    const auto start = std::chrono::steady_clock::now();
    const bool ok = sub->deserialize( *serialized, compound );
    const auto end = std::chrono::steady_clock::now();
    if ( !ok )
      return; // deserialization failure is already logged by ros_babel_fish
    const int64_t deserialize_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>( end - start ).count();
    stats.deserialize_ns.add( deserialize_ns );

    bool has_latency = false;
    int64_t latency_ns = 0;
    rclcpp::Time stamp;
    if ( try_get_stamp( compound, stamp ) && stamp.nanoseconds() != 0 ) {
      // Subtract raw nanoseconds to avoid clock-type mismatch between the node clock and the
      // clock the header was stamped with.
      latency_ns = recv.nanoseconds() - stamp.nanoseconds();
      has_latency = true;
      stats.latency_ns.add( latency_ns );
    } else if ( !latency_warned ) {
      // Bandwidth and deserialize time are still measured; only latency needs a header stamp.
      std::cerr << "Topic '" << topic << "' message type '" << sub->get_message_type()
                << "' has no usable 'header' stamp; latency will not be measured." << std::endl;
      latency_warned = true;
    }

    if ( csv.is_open() ) {
      csv << recv.nanoseconds() << ',';
      if ( has_latency )
        csv << latency_ns;
      csv << ',' << deserialize_ns << ',' << size << '\n';
    }
  };

  try {
    // Subscribe best effort to stay compatible with best-effort publishers (e.g. sensor topics).
    rclcpp::QoS qos = rclcpp::QoS( rclcpp::KeepLast( 50 ) ).best_effort();
    if ( type.empty() ) {
      sub = fish.create_subscription( *node, topic, qos, callback );
    } else {
      sub = fish.create_subscription( *node, topic, type, qos, callback );
    }
  } catch ( const std::exception &e ) {
    std::cerr << "Failed to create subscription: " << e.what() << std::endl;
    return 1;
  }

  if ( !sub ) {
    std::cerr << "Could not create subscription!" << std::endl;
    return 1;
  }

  std::cout << "Monitoring '" << topic << "' [" << sub->get_message_type() << "], reporting every "
            << window << "s ...";
  if ( csv.is_open() )
    std::cout << " writing measurements to '" << out_path << "'";
  std::cout << std::endl;

  auto last_report = std::chrono::steady_clock::now();
  auto report = [&stats, &last_report, &csv]() {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>( now - last_report ).count();
    last_report = now;

    if ( csv.is_open() )
      csv.flush();

    if ( stats.message_count == 0 ) {
      std::cout << "no messages" << std::endl;
      stats.reset();
      return;
    }

    std::ostringstream line;
    line << stats.message_count << " msgs (" << std::fixed << std::setprecision( 1 )
         << stats.message_count / elapsed << " Hz)";

    line << " | latency ms min/avg/max ";
    if ( stats.latency_ns.count == 0 ) {
      line << "n/a";
    } else {
      line << std::setprecision( 2 ) << stats.latency_ns.min / 1e6 << "/"
           << stats.latency_ns.avg() / 1e6 << "/" << stats.latency_ns.max / 1e6;
    }

    line << " | deser us min/avg/max ";
    if ( stats.deserialize_ns.count == 0 ) {
      line << "n/a";
    } else {
      line << std::setprecision( 1 ) << stats.deserialize_ns.min / 1e3 << "/"
           << stats.deserialize_ns.avg() / 1e3 << "/" << stats.deserialize_ns.max / 1e3;
    }

    line << " | bw " << format_bytes_per_sec( stats.total_bytes / elapsed );
    std::cout << line.str() << std::endl;
    stats.reset();
  };

  auto period = std::chrono::nanoseconds( static_cast<int64_t>( window * 1e9 ) );
  auto timer = node->create_wall_timer( period, report );

  rclcpp::spin( node );

  return 0;
}
