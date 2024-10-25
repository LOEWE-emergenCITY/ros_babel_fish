//
// Created by Stefan Fabian on 03.03.20.
//

#include "message_comparison.hpp"
#include <ros_babel_fish/babel_fish.hpp>

#include <action_msgs/msg/goal_status_array.h>
#include <example_interfaces/action/fibonacci.hpp>
#include <ros_babel_fish_test_msgs/action/simple_test.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rosidl_typesupport_c/type_support_map.h>

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

TEST( ActionClientTest, actionLookup )
{
  BabelFish fish;
  ActionTypeSupport::ConstSharedPtr type_support =
      fish.get_action_type_support( "ros_babel_fish_test_msgs/action/SimpleTest" );
  ASSERT_NE( type_support, nullptr );

  const rosidl_action_type_support_t *ts = rosidl_typesupport_cpp::get_action_type_support_handle<
      ros_babel_fish_test_msgs::action::SimpleTest>();
  EXPECT_NE( ts, nullptr );
  EXPECT_TRUE( Equal( type_support->type_support_handle, *ts ) );

  type_support = fish.get_action_type_support( "example_interfaces/action/Fibonacci" );
  ASSERT_NE( type_support, nullptr );

  const auto *ts_map =
      static_cast<const type_support_map_t *>( ts->status_message_type_support->data );
  const auto *map = static_cast<const type_support_map_t *>(
      type_support->status_message_type_support->type_support_handle.data );
  EXPECT_EQ( ts_map, map );
}

TEST( ActionClientTest, actionClient )
{
  using namespace std::chrono_literals;
  using Action = ros_babel_fish_test_msgs::action::SimpleTest;
  rclcpp_action::Server<Action>::SharedPtr server = rclcpp_action::create_server<Action>(
      node, "ros_babel_fish_client_test_server",
      []( const rclcpp_action::GoalUUID &, const std::shared_ptr<const Action::Goal> &goal ) {
        if ( goal->target == 13 ) // 13 is rejected
          return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      []( const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> &handle ) {
        if ( handle->get_goal()->target == 7 ) // 7 can't be canceled
          return rclcpp_action::CancelResponse::REJECT;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      []( const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> &handle ) {
        std::thread t( [handle]() {
          auto result = std::make_shared<Action::Result>();
          for ( int i = 0; i < handle->get_goal()->target; ++i ) {
            if ( i >= 10 && i >= handle->get_goal()->target / 2 ) {
              result->final_value = i;
              handle->abort( result );
              return;
            }
            std::this_thread::sleep_for( 50ms );
            auto feedback = std::make_shared<Action::Feedback>();
            feedback->current_value = i;
            handle->publish_feedback( feedback );
            if ( handle->is_canceling() ) {
              result->final_value = i;
              handle->canceled( result );
              return;
            }
          }
          result->final_value = handle->get_goal()->target;
          handle->succeed( result );
        } );
        t.detach();
      } );

  BabelFish fish;
  auto client = fish.create_action_client( *node, "ros_babel_fish_client_test_server",
                                           "ros_babel_fish_test_msgs/action/SimpleTest" );
  ASSERT_TRUE( client->wait_for_action_server( 5s ) );
  ASSERT_TRUE( client->action_server_is_ready() );

  // This goal should succeed
  auto goal = client->create_goal();
  goal["target"] = 2;
  auto gh_future = client->async_send_goal( goal );
  ASSERT_EQ( gh_future.wait_for( 30s ), std::future_status::ready );
  auto goal_handle = gh_future.get();
  ASSERT_NE( goal_handle, nullptr );
  ASSERT_EQ( goal_handle->get_status(), action_msgs::msg::GoalStatus::STATUS_ACCEPTED );
  auto result = client->async_get_result( goal_handle );
  ASSERT_EQ( result.wait_for( 3s ), std::future_status::ready );
  ASSERT_EQ( goal_handle->get_status(), action_msgs::msg::GoalStatus::STATUS_SUCCEEDED );
  auto result_msg = result.get();
  EXPECT_EQ( ( *result_msg.result )["final_value"].value<int32_t>(), 2 );

  // This goal should abort after 10
  goal = client->create_goal();
  goal["target"] = 20;
  std::vector<int> feedback_values;
  gh_future = client->async_send_goal(
      goal, { {},
              [&feedback_values]( const BabelFishActionClient::GoalHandle::SharedPtr &,
                                  const CompoundMessage::ConstSharedPtr &feedback ) {
                feedback_values.push_back( ( *feedback )["current_value"].value<int32_t>() );
              },
              {} } );
  ASSERT_EQ( gh_future.wait_for( 3s ), std::future_status::ready );
  goal_handle = gh_future.get();
  ASSERT_NE( goal_handle, nullptr );
  ASSERT_EQ( goal_handle->get_status(), action_msgs::msg::GoalStatus::STATUS_ACCEPTED );
  result = client->async_get_result( goal_handle );
  ASSERT_EQ( result.wait_for( 10s ), std::future_status::ready );
  ASSERT_EQ( goal_handle->get_status(), action_msgs::msg::GoalStatus::STATUS_ABORTED );
  result_msg = result.get();
  EXPECT_EQ( ( *result_msg.result )["final_value"].value<int32_t>(), 10 );

  ASSERT_EQ( feedback_values.size(), 10U );
  for ( int i = 0; i < 10; ++i ) {
    if ( feedback_values[i] != i ) // cppcheck-suppress containerOutOfBounds
      FAIL() << "Feedback at " << i << " should be " << i << "!";
  }

  // This goal should be preempted
  goal = client->create_goal();
  goal["target"] = 200;
  gh_future = client->async_send_goal( goal );
  ASSERT_EQ( gh_future.wait_for( 3s ), std::future_status::ready );
  goal_handle = gh_future.get();
  ASSERT_NE( goal_handle, nullptr );
  std::this_thread::sleep_for( 200ms );
  auto cancel_response_future = client->async_cancel_goal( goal_handle );
  ASSERT_EQ( cancel_response_future.wait_for( 1s ), std::future_status::ready )
      << "ActionServer did not cancel in 1 second!";
  auto cancel_response = cancel_response_future.get();
  ASSERT_EQ( cancel_response["return_code"].value<int8_t>(),
             action_msgs::srv::CancelGoal::Response::ERROR_NONE );
  result = client->async_get_result( goal_handle );
  ASSERT_EQ( result.wait_for( 10s ), std::future_status::ready );
  EXPECT_EQ( goal_handle->get_status(), action_msgs::msg::GoalStatus::STATUS_CANCELED );
  result_msg = result.get();
  EXPECT_LT( ( *result_msg.result )["final_value"].value<int32_t>(), 200 );

  // 13 should be rejected
  goal = client->create_goal();
  goal["target"] = 13;
  gh_future = client->async_send_goal( goal );
  ASSERT_EQ( gh_future.wait_for( 3s ), std::future_status::ready );
  goal_handle = gh_future.get();
  ASSERT_EQ( goal_handle, nullptr );

  // 7 can't be canceled
  goal = client->create_goal();
  goal["target"] = 7;
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
  ASSERT_EQ( cancel_response["return_code"].value<int8_t>(),
             action_msgs::srv::CancelGoal::Response::ERROR_REJECTED );
  result = client->async_get_result( goal_handle );
  ASSERT_EQ( result.wait_for( 10s ), std::future_status::ready );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  node = std::make_shared<rclcpp::Node>( "action_client_test" );
  std::thread spinner( []() { rclcpp::spin( node ); } );
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  spinner.join();
  node.reset();
  return result;
}
