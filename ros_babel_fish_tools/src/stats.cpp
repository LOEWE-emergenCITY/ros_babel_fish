// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "compression.hpp"

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
#include <string>
#include <vector>

using namespace ros_babel_fish;
using namespace ros_babel_fish_tools;

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
  std::cerr << "  --compress <algo>   Also report compressed bandwidth and (de)compression time;"
            << std::endl;
  std::cerr << "                      algo is 'lz4' or 'zstd'" << std::endl;
  std::cerr << "  --ros-args ...      Pass ROS arguments (e.g. -p use_sim_time:=true)" << std::endl;
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
  Accumulator compress_ns;    //!< time spent compressing the serialized message
  Accumulator decompress_ns;  //!< time spent decompressing it again
  uint64_t message_count = 0;
  uint64_t total_bytes = 0;
  uint64_t total_compressed_bytes = 0;

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

//! Appends "min/avg/max" for @p acc to @p os (each value divided by @p divisor), or "n/a" if no
//! samples were collected. Assumes @p os already has the desired float formatting (e.g. std::fixed).
void append_min_avg_max( std::ostream &os, const Accumulator &acc, double divisor, int precision )
{
  if ( acc.count == 0 ) {
    os << "n/a";
    return;
  }
  os << std::setprecision( precision ) << acc.min / divisor << "/" << acc.avg() / divisor << "/"
     << acc.max / divisor;
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
  // Strip ROS arguments (e.g. --ros-args -p use_sim_time:=true) before our own parsing; the node
  // picks them up automatically via the global context.
  const std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments( argc, argv );

  std::string topic;
  std::string type;
  std::string out_path;
  double window = 5.0;
  CompressionAlgorithm compression = CompressionAlgorithm::None;
  for ( size_t i = 1; i < args.size(); ++i ) {
    const std::string &arg = args[i];
    if ( arg == "-h" || arg == "--help" ) {
      print_usage( argv[0] );
      return 0;
    }
    if ( arg == "--window" ) {
      if ( i + 1 >= args.size() ) {
        std::cerr << "Missing value for --window" << std::endl;
        print_usage( argv[0] );
        return 1;
      }
      const std::string &value = args[++i];
      char *end = nullptr;
      window = std::strtod( value.c_str(), &end );
      // Reject junk ("5x"), empty parses and values large enough to overflow the timer period.
      if ( end == value.c_str() || *end != '\0' || !std::isfinite( window ) || window <= 0.0 ||
           window > static_cast<double>( std::numeric_limits<int64_t>::max() ) / 1e9 ) {
        std::cerr << "--window must be a positive number of seconds" << std::endl;
        return 1;
      }
    } else if ( arg == "--out" ) {
      if ( i + 1 >= args.size() ) {
        std::cerr << "Missing value for --out" << std::endl;
        print_usage( argv[0] );
        return 1;
      }
      out_path = args[++i];
    } else if ( arg == "--compress" ) {
      if ( i + 1 >= args.size() ) {
        std::cerr << "Missing value for --compress" << std::endl;
        print_usage( argv[0] );
        return 1;
      }
      const std::string &value = args[++i];
      if ( !parse_compression_algorithm( value, compression ) ) {
        std::cerr << "--compress must be 'lz4' or 'zstd', got: " << value << std::endl;
        return 1;
      }
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
    csv << "recv_ns,latency_ns,deserialize_ns,size_bytes";
    if ( compression != CompressionAlgorithm::None )
      csv << ",compress_ns,decompress_ns,compressed_size_bytes";
    csv << '\n';
  }

  auto node = std::make_shared<rclcpp::Node>( "ros_babel_fish_stats" );
  BabelFish fish;

  WindowStats stats;
  bool latency_warned = false;
  bool compress_warned = false;
  BabelFishSubscription::SharedPtr sub;

  // Reused across messages as the compression/decompression destination buffers (the callback runs
  // single-threaded under rclcpp::spin, so no synchronization is needed).
  std::vector<char> compress_scratch;
  std::vector<char> decompress_scratch;

  // A serialized-message callback makes BabelFish deliver the raw bytes, which gives us the
  // message size for bandwidth and lets us time the deserialization ourselves.
  auto callback = [&sub, &stats, &latency_warned, &compress_warned, &csv, &compress_scratch,
                   &decompress_scratch, compression, node = node.get(),
                   topic]( std::shared_ptr<rclcpp::SerializedMessage> serialized ) {
    const rclcpp::Time recv = node->now();
    const size_t size = serialized->size();
    stats.total_bytes += size;
    stats.message_count += 1;

    // Compress before deserializing so a deserialization failure does not skew the compressed
    // bandwidth relative to the raw bandwidth (which counts this message either way).
    int64_t compress_ns = 0;
    int64_t decompress_ns = 0;
    size_t compressed_bytes = 0;
    if ( compression != CompressionAlgorithm::None ) {
      const uint8_t *raw = serialized->get_rcl_serialized_message().buffer;
      try {
        // Size the destination buffers up front so a first-message (or growth) allocation is not
        // charged to the measured compression/decompression time.
        const size_t bound = compress_bound( compression, size );
        if ( compress_scratch.size() < bound )
          compress_scratch.resize( bound );
        if ( decompress_scratch.size() < size )
          decompress_scratch.resize( size );

        const auto cstart = std::chrono::steady_clock::now();
        compressed_bytes = compress( compression, raw, size, compress_scratch );
        const auto cend = std::chrono::steady_clock::now();
        decompress( compression, compress_scratch.data(), compressed_bytes, size, decompress_scratch );
        const auto dend = std::chrono::steady_clock::now();
        compress_ns = std::chrono::duration_cast<std::chrono::nanoseconds>( cend - cstart ).count();
        decompress_ns = std::chrono::duration_cast<std::chrono::nanoseconds>( dend - cend ).count();
        stats.total_compressed_bytes += compressed_bytes;
        stats.compress_ns.add( compress_ns );
        stats.decompress_ns.add( decompress_ns );
      } catch ( const std::exception &e ) {
        // A (de)compression failure must not take down the monitor; skip this message's
        // measurement, warn once, and keep going (mirrors the deserialize-failure handling below).
        if ( !compress_warned ) {
          std::cerr << "Compression failed for a message on '" << topic << "': " << e.what()
                    << "; skipping its (de)compression measurement." << std::endl;
          compress_warned = true;
        }
        return;
      }
    }

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
      csv << ',' << deserialize_ns << ',' << size;
      if ( compression != CompressionAlgorithm::None )
        csv << ',' << compress_ns << ',' << decompress_ns << ',' << compressed_bytes;
      csv << '\n';
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
  if ( compression != CompressionAlgorithm::None )
    std::cout << " compressing with " << compression_name( compression );
  if ( csv.is_open() )
    std::cout << " writing measurements to '" << out_path << "'";
  std::cout << std::endl;

  auto last_report = std::chrono::steady_clock::now();
  auto report = [&stats, &last_report, &csv, compression]() {
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
    append_min_avg_max( line, stats.latency_ns, 1e6, 2 );

    line << " | deser us min/avg/max ";
    append_min_avg_max( line, stats.deserialize_ns, 1e3, 1 );

    line << " | bw " << format_bytes_per_sec( stats.total_bytes / elapsed );
    std::cout << line.str() << std::endl;

    // Compression metrics go on their own indented line to keep the main line readable.
    if ( compression != CompressionAlgorithm::None ) {
      std::ostringstream cline;
      cline << std::fixed << "  " << compression_name( compression ) << " bw "
            << format_bytes_per_sec( stats.total_compressed_bytes / elapsed );
      if ( stats.total_compressed_bytes != 0 ) {
        cline << " (" << std::setprecision( 2 )
              << static_cast<double>( stats.total_bytes ) / stats.total_compressed_bytes << "x)";
      }
      cline << " | compress us min/avg/max ";
      append_min_avg_max( cline, stats.compress_ns, 1e3, 1 );
      cline << " | decompress us min/avg/max ";
      append_min_avg_max( cline, stats.decompress_ns, 1e3, 1 );
      std::cout << cline.str() << std::endl;
    }

    stats.reset();
  };

  auto period = std::chrono::nanoseconds( static_cast<int64_t>( window * 1e9 ) );
  auto timer = node->create_wall_timer( period, report );

  rclcpp::spin( node );

  return 0;
}
