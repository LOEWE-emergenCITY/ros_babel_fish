//
// Created by Stefan Fabian on 04.09.19.
//

// #include "message_comparison.h"
#include <gtest/gtest.h>

#include <ros_babel_fish/babel_fish.hpp>

#include <example_interfaces/srv/add_two_ints.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace ros_babel_fish;
using namespace std::chrono_literals;

std::shared_ptr<rclcpp::Node> node;

void twoIntServiceCallback( const example_interfaces::srv::AddTwoInts::Request::SharedPtr req,
                            example_interfaces::srv::AddTwoInts::Response::SharedPtr resp )
{
  resp->sum = req->a + req->b + 42;
}

TEST( ServiceClientTest, tests )
{
  BabelFish fish;
  example_interfaces::srv::AddTwoInts_Request req2;
  req2.a = 512;
  req2.b = 314;
  fish.create_message_shared( "ros_babel_fish_test_msgs/TestMessage" );
  CompoundMessage::SharedPtr req =
      fish.create_service_request_shared( "example_interfaces/srv/AddTwoInts" );
  ( *req )["a"] = 512;
  ( *req )["b"] = 314;
  BabelFishServiceClient::SharedPtr client = fish.create_service_client(
      *node, "/test_service_client/two_ints", "example_interfaces/srv/AddTwoInts" );
  ASSERT_TRUE( client->wait_for_service( 5s ) );
  std::shared_future<CompoundMessage::SharedPtr> response_future = client->async_send_request( req );
  ASSERT_EQ( response_future.wait_for( 5s ), std::future_status::ready );
  CompoundMessage::SharedPtr response = response_future.get();
  ASSERT_NE( response->type_erased_message(), nullptr );
  ASSERT_TRUE( response->containsKey( "sum" ) );
  EXPECT_EQ( ( *response )["sum"].value<int64_t>(), 868 ); // Sum is 512 + 314 + 42 = 868
}

TEST( ServiceTest, server )
{
  //  ros::service::waitForService( "/test_service_server/subscribers" );
  //  rosapi::Subscribers subscribers;
  //  subscribers.request_template.topic = "First test";
  //  EXPECT_TRUE( ros::service::call( "/test_service_server/subscribers", subscribers ));
  //  ASSERT_EQ( subscribers.response.subscribers.size(), 2UL );
  //  EXPECT_EQ( subscribers.response.subscribers[0], "First test" );
  //  EXPECT_EQ( subscribers.response.subscribers[1], "The answer to everything is:" );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  node = std::make_shared<rclcpp::Node>( "service_client_test" );
  std::thread spinner( []() { rclcpp::spin( node ); } );
  auto service_two_ints = node->create_service<example_interfaces::srv::AddTwoInts>(
      "/test_service_client/two_ints", &twoIntServiceCallback );
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  spinner.join();
  node.reset();
  return result;
}
