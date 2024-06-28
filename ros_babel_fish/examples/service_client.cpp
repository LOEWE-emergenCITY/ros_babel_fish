// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <rclcpp/rclcpp.hpp>
#include <ros_babel_fish/babel_fish.hpp>
#include <ros_babel_fish/messages/message_types.hpp>

using namespace ros_babel_fish;
using namespace std::chrono_literals;

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  auto node = rclcpp::Node::make_shared( "service_client" );
  BabelFish fish;
  CompoundMessage::SharedPtr req =
      fish.create_service_request_shared( "example_interfaces/srv/AddTwoInts" );
  ( *req )["a"] = 314;
  ( *req )["b"] = 1337;
  BabelFishServiceClient::SharedPtr client = fish.create_service_client(
      *node, "/ros_babel_fish/service", "example_interfaces/srv/AddTwoInts" );
  RCLCPP_INFO( node->get_logger(), "Waiting for server." );
  if ( !client->wait_for_service( 5s ) ) {
    std::cerr << "Timeout while waiting for service!" << std::endl;
    return 1;
  }
  RCLCPP_INFO( node->get_logger(), "Service available. Sending request." );
  std::shared_future<CompoundMessage::SharedPtr> future = client->async_send_request( req );
  if ( rclcpp::spin_until_future_complete( node, future, 5s ) != rclcpp::FutureReturnCode::SUCCESS ) {
    std::cerr << "Timeout while waiting for reply from service!" << std::endl;
    return 1;
  }
  std::cout << "Response:" << std::endl;
  std::cout << "  sum: " << ( *future.get() )["sum"].value<int64_t>() << std::endl;
  return 0;
}
