// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <ros2_babel_fish/babel_fish.hpp>
#include <ros2_babel_fish/messages/message_types.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace ros2_babel_fish;

class ServiceServer : public rclcpp::Node
{
public:
  ServiceServer() : Node( "service_server" ) { }

  void init()
  {
    service_ = fish_.create_service( *this, "/ros2_babel_fish/service", "example_interfaces/srv/AddTwoInts",
                                     []( const CompoundMessage::SharedPtr request,
                                         CompoundMessage::SharedPtr response ) -> bool
                                     {
                                       std::cout << "Received request: " << std::endl;
                                       std::cout << "a: " << (*request)["a"].value<int64_t>() << std::endl;
                                       std::cout << "b: " << (*request)["b"].value<int64_t>() << std::endl;
                                       (*response)["sum"] = 42;
                                       return true;
                                     } );
    RCLCPP_INFO( get_logger(), "Service ready." );
  }

private:
  BabelFishService::SharedPtr service_;
  BabelFish fish_;
};

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  auto node = std::make_shared<ServiceServer>();
  node->init();
  rclcpp::spin( node );
  return 0;
}
