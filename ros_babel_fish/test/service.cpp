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
      *node, "/test_service_client/two_ints_client", "example_interfaces/srv/AddTwoInts" );
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
  BabelFish fish;
  auto server = fish.create_service(
      *node, "/test_service_server/two_ints_server", "example_interfaces/srv/AddTwoInts",
      []( CompoundMessage::SharedPtr req, CompoundMessage::SharedPtr resp ) {
        resp->set( "sum", req->get<int64_t>( "a" ) + req->get<int64_t>( "b" ) + 1337 );
        return true;
      } );
  auto req2 = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  req2->a = 512;
  req2->b = 314;
  auto client = node->create_client<example_interfaces::srv::AddTwoInts>(
      "test_service_server/two_ints_server" );
  ASSERT_TRUE( client->wait_for_service( 5s ) );
  auto response = client->async_send_request( req2 );
  ASSERT_TRUE( response.wait_for( 5s ) == std::future_status::ready );
  auto result = response.get();
  ASSERT_NE( result, nullptr );
  EXPECT_EQ( result->sum, 512 + 314 + 1337 );
}

TEST( ServiceTest, deferredResponse )
{
  BabelFish fish;
  auto type_support = fish.get_service_type_support( "example_interfaces/srv/AddTwoInts" );
  std::shared_ptr<rmw_request_id_t> stored_header;
  CompoundMessage::SharedPtr stored_request;
  rclcpp::TimerBase::SharedPtr send_timer;
  BabelFishService::SharedPtr service;
  // Register the defer variant (header, request) which must NOT auto-send. The response is sent
  // later, back on the spinner thread, via a one-shot wall timer so take_request and send_response
  // stay on the same thread.
  service = fish.create_service(
      *node, "/test_service_server/deferred_two_ints", "example_interfaces/srv/AddTwoInts",
      [&]( std::shared_ptr<rmw_request_id_t> header, CompoundMessage::SharedPtr request ) {
        stored_header = std::move( header );
        stored_request = std::move( request );
        send_timer = node->create_wall_timer( 250ms, [&]() {
          send_timer->cancel();
          auto response = CompoundMessage::make_shared( type_support->response() );
          response->set( "sum", stored_request->get<int64_t>( "a" ) +
                                    stored_request->get<int64_t>( "b" ) + 100 );
          service->send_response( *stored_header, *response );
        } );
      } );
  auto req = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  req->a = 7;
  req->b = 5;
  auto client = node->create_client<example_interfaces::srv::AddTwoInts>(
      "test_service_server/deferred_two_ints" );
  ASSERT_TRUE( client->wait_for_service( 5s ) );
  auto response_future = client->async_send_request( req );
  // Regression guard: with the auto-send disabled for defer variants, the client must not receive
  // an (empty) response before the deferred send actually fires.
  EXPECT_EQ( response_future.wait_for( 100ms ), std::future_status::timeout );
  ASSERT_EQ( response_future.wait_for( 5s ), std::future_status::ready );
  auto result = response_future.get();
  ASSERT_NE( result, nullptr );
  EXPECT_EQ( result->sum, 7 + 5 + 100 );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  node = std::make_shared<rclcpp::Node>( "service_client_test" );
  std::thread spinner( []() { rclcpp::spin( node ); } );
  auto service_two_ints = node->create_service<example_interfaces::srv::AddTwoInts>(
      "/test_service_client/two_ints_client", &twoIntServiceCallback );
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  spinner.join();
  node.reset();
  return result;
}
