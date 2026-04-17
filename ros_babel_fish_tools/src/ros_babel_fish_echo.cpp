// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish_tools/nlohmann_json_serialization.hpp"
#include "ros_babel_fish_tools/yaml_cpp_serialization.hpp"

#include <cstdlib>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

using namespace ros_babel_fish;
using namespace ros_babel_fish_tools;

void print_usage( const char *name )
{
  std::cerr << "Usage: " << name << " <topic> [type] [options]" << std::endl;
  std::cerr << "Options:" << std::endl;
  std::cerr << "  -h, --help    Show this help message" << std::endl;
  std::cerr << "  --json        Output as JSON (default)" << std::endl;
  std::cerr << "  --yaml        Output as YAML" << std::endl;
  std::cerr << "  --pretty, -p  Pretty print the output" << std::endl;
}

int main( int argc, char **argv )
{
  // Turn off Zenoh logging to avoid additional zenoh output.
  // This node should only output the message, so any additional output is undesirable.
#ifdef _WIN32
  _putenv_s( "RUST_LOG", "off" );
#else
  setenv( "RUST_LOG", "off", 1 );
#endif
  rclcpp::init( argc, argv );

  std::string topic;
  std::string type;
  bool output_json = true;
  bool pretty = false;
  for ( int i = 1; i < argc; ++i ) {
    std::string arg = argv[i];
    if ( arg == "-h" || arg == "--help" ) {
      print_usage( argv[0] );
      return 0;
    }
    if ( arg == "--json" ) {
      output_json = true;
    } else if ( arg == "--yaml" ) {
      output_json = false;
    } else if ( arg == "--pretty" || arg == "-p" ) {
      pretty = true;
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

  auto node = std::make_shared<rclcpp::Node>( "ros_babel_fish_echo" );
  BabelFish fish;

  BabelFishSubscription::SharedPtr sub;

  auto callback = [output_json, pretty]( const CompoundMessage &msg ) {
    if ( output_json ) {
      json j = message_to_json( msg );
      if ( pretty )
        std::cout << j.dump( 4 ) << std::endl;
      else
        std::cout << j.dump() << std::endl;
    } else {
      YAML::Node n = message_to_yaml( msg );
      std::cout << n << std::endl;
    }
    rclcpp::shutdown();
  };

  try {
    if ( type.empty() ) {
      sub = fish.create_subscription( *node, topic, rclcpp::QoS( 1 ), callback );
    } else {
      sub = fish.create_subscription( *node, topic, type, rclcpp::QoS( 1 ), callback );
    }
  } catch ( const std::exception &e ) {
    std::cerr << "Failed to create subscription: " << e.what() << std::endl;
    return 1;
  }

  if ( !sub ) {
    std::cerr << "Could not create subscription!" << std::endl;
    return 1;
  }

  rclcpp::spin( node );

  return 0;
}
