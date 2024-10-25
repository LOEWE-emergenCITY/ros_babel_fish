// Copyright (c) 2024 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "message_comparison.hpp"
#include <ros_babel_fish/babel_fish.hpp>

#include <action_msgs/msg/goal_status_array.h>
#include <example_interfaces/action/fibonacci.hpp>
#include <ros_babel_fish_test_msgs/action/simple_test.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace ros_babel_fish;
using namespace ros_babel_fish_test_msgs;

rclcpp::Node::SharedPtr node;

::testing::AssertionResult Equal( const rosidl_message_type_support_t &a,
                                  const rosidl_message_type_support_t &b )
{
  if ( a.data != b.data )
    return ::testing::AssertionFailure() << "Type support maps are not equal!";
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult Equal( const rosidl_action_type_support_t &a,
                                  const rosidl_action_type_support_t &b )
{
  ::testing::AssertionResult result = ::testing::AssertionSuccess();
  if ( !( result = Equal( *a.status_message_type_support, *b.status_message_type_support ) ) )
    return result << std::endl << "Action/StatusMessage";
  if ( !( result = Equal( *a.feedback_message_type_support, *b.feedback_message_type_support ) ) )
    return result << std::endl << "Action/StatusMessage";
  return result;
}

TEST( ActionClientTest, actionServer )
{
  using namespace std::chrono_literals;
  using Action = ros_babel_fish_test_msgs::action::SimpleTest;
  BabelFish fish;
  auto server = fish.create_action_server(
      *node, "ros_babel_fish_server_test_server", "ros_babel_fish_test_msgs/action/SimpleTest",
      []( const rclcpp_action::GoalUUID &, const CompoundMessage::ConstSharedPtr &msg ) {
        if ( ( *msg )["target"] == 13 ) // 13 is rejected
          return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      []( const std::shared_ptr<BabelFishActionServerGoalHandle> &handle ) {
        if ( ( *handle->get_goal() )["target"].value<int32_t>() == 7 )
          return rclcpp_action::CancelResponse::REJECT;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      []( const std::shared_ptr<BabelFishActionServerGoalHandle> &handle ) {
        std::thread t( [handle]() {
          auto result = handle->create_result_message();
          int goal = ( *handle->get_goal() )["target"].value<int32_t>();
          for ( int i = 0; i < goal; ++i ) {
            if ( i >= 10 && i >= goal / 2 ) {
              result["final_value"] = i;
              handle->abort( result );
              return;
            }
            std::this_thread::sleep_for( 50ms );
            auto feedback = handle->create_feedback_message();
            feedback["current_value"] = i;
            handle->publish_feedback( feedback );
            if ( handle->is_canceling() ) {
              result["final_value"] = i;
              handle->canceled( result );
              return;
            }
          }
          result["final_value"] = goal;
          handle->succeed( result );
        } );
        t.detach();
      } );
  auto client = rclcpp_action::create_client<Action>( node, "ros_babel_fish_server_test_server" );
  ASSERT_TRUE( client->wait_for_action_server( 5s ) );
  ASSERT_TRUE( client->action_server_is_ready() );

  // This goal should succeed
  Action::Goal goal;
  goal.target = 2;
  auto gh_future = client->async_send_goal( goal );
  ASSERT_EQ( gh_future.wait_for( 30s ), std::future_status::ready );
  auto goal_handle = gh_future.get();
  ASSERT_NE( goal_handle, nullptr );
  ASSERT_EQ( goal_handle->get_status(), action_msgs::msg::GoalStatus::STATUS_ACCEPTED );
  auto result = client->async_get_result( goal_handle );
  ASSERT_EQ( result.wait_for( 3s ), std::future_status::ready );
  ASSERT_EQ( goal_handle->get_status(), action_msgs::msg::GoalStatus::STATUS_SUCCEEDED );
  auto result_msg = result.get();
  EXPECT_EQ( result_msg.result->final_value, 2 );

  // This goal should abort after 10
  goal.target = 20;
  std::vector<int> feedback_values;
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.feedback_callback =
      [&feedback_values]( auto, const std::shared_ptr<const Action::Feedback> &feedback ) {
        feedback_values.push_back( feedback->current_value );
      };
  gh_future = client->async_send_goal( goal, options );
  ASSERT_EQ( gh_future.wait_for( 3s ), std::future_status::ready );
  goal_handle = gh_future.get();
  ASSERT_NE( goal_handle, nullptr );
  ASSERT_EQ( goal_handle->get_status(), action_msgs::msg::GoalStatus::STATUS_ACCEPTED );
  result = client->async_get_result( goal_handle );
  ASSERT_EQ( result.wait_for( 10s ), std::future_status::ready );
  ASSERT_EQ( goal_handle->get_status(), action_msgs::msg::GoalStatus::STATUS_ABORTED );
  result_msg = result.get();
  EXPECT_EQ( result_msg.result->final_value, 10 );

  ASSERT_EQ( feedback_values.size(), 10U );
  for ( int i = 0; i < 10; ++i ) {
    if ( feedback_values[i] != i ) // cppcheck-suppress containerOutOfBounds
      FAIL() << "Feedback at " << i << " should be " << i << "!";
  }

  // This goal should be preempted
  goal.target = 200;
  gh_future = client->async_send_goal( goal );
  ASSERT_EQ( gh_future.wait_for( 3s ), std::future_status::ready );
  goal_handle = gh_future.get();
  ASSERT_NE( goal_handle, nullptr );
  std::this_thread::sleep_for( 200ms );
  auto cancel_response_future = client->async_cancel_goal( goal_handle );
  ASSERT_EQ( cancel_response_future.wait_for( 1s ), std::future_status::ready )
      << "ActionServer did not cancel in 1 second!";
  auto cancel_response = cancel_response_future.get();
  ASSERT_EQ( cancel_response->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE );
  result = client->async_get_result( goal_handle );
  ASSERT_EQ( result.wait_for( 10s ), std::future_status::ready );
  EXPECT_EQ( goal_handle->get_status(), action_msgs::msg::GoalStatus::STATUS_CANCELED );
  result_msg = result.get();
  EXPECT_LT( result_msg.result->final_value, 200 );

  // 13 should be rejected
  goal.target = 13;
  gh_future = client->async_send_goal( goal );
  ASSERT_EQ( gh_future.wait_for( 3s ), std::future_status::ready );
  goal_handle = gh_future.get();
  ASSERT_EQ( goal_handle, nullptr );

  // 7 can't be canceled
  goal.target = 7;
  gh_future = client->async_send_goal( goal );
  ASSERT_EQ( gh_future.wait_for( 3s ), std::future_status::ready );
  goal_handle = gh_future.get();
  ASSERT_NE( goal_handle, nullptr );
  ASSERT_EQ( goal_handle->get_status(), action_msgs::msg::GoalStatus::STATUS_ACCEPTED );
  std::this_thread::sleep_for( 50ms );
  cancel_response_future = client->async_cancel_goal( goal_handle );
  ASSERT_EQ( cancel_response_future.wait_for( 1s ), std::future_status::ready )
      << "ActionServer did not cancel in 1 second!";
  cancel_response = cancel_response_future.get();
  ASSERT_EQ( cancel_response->return_code, action_msgs::srv::CancelGoal::Response::ERROR_REJECTED );
  result = client->async_get_result( goal_handle );
  ASSERT_EQ( result.wait_for( 10s ), std::future_status::ready );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  node = std::make_shared<rclcpp::Node>( "action_server_test" );
  std::thread spinner( []() { rclcpp::spin( node ); } );
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  spinner.join();
  node.reset();
  return result;
}
