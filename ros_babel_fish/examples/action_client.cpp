// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros_babel_fish/babel_fish.hpp>

using namespace ros_babel_fish;
using namespace std::chrono_literals;

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  auto node = rclcpp::Node::make_shared( "action_client" );
  std::thread spin_thread( [node]() { rclcpp::spin( node ); } );
  BabelFish fish;
  auto client = fish.create_action_client( *node, "fibonacci",
                                           "action_tutorials_interfaces/action/Fibonacci" );

  RCLCPP_INFO( node->get_logger(), "Waiting for server to come up." );
  if ( !client->wait_for_action_server( 10s ) ) {
    RCLCPP_ERROR( node->get_logger(), "Timeout after 10s. Could not connect to server!" );
    return 1;
  }
  // Create and send goal
  CompoundMessage goal = client->create_goal();
  goal["order"] = 7;
  RCLCPP_INFO( node->get_logger(), "Sending goal." );
  std::shared_future<BabelFishActionClient::GoalHandle::SharedPtr> goal_handle_future =
      client->async_send_goal(
          goal,
          {
              nullptr, // We handle the goal response using the future
              []( BabelFishActionClient::GoalHandle::SharedPtr,
                  const CompoundMessage::ConstSharedPtr msg ) -> void {
                const auto &sequence = ( *msg )["partial_sequence"].as<ArrayMessage<int32_t>>();
                std::cout << "Feedback is an array of length:" << sequence.size() << std::endl;
                for ( size_t i = 0; i < sequence.size(); ++i ) {
                  std::cout << sequence[i];
                  if ( i != sequence.size() - 1 )
                    std::cout << ", ";
                }
                std::cout << std::endl;
              },
              nullptr // Result is also handled with future
          } );
  BabelFishActionClient::GoalHandle::SharedPtr goal_handle = goal_handle_future.get();
  if ( goal_handle == nullptr ) {
    RCLCPP_ERROR( node->get_logger(), "Goal was not accepted!" );
    return 1;
  }
  RCLCPP_INFO( node->get_logger(), "Goal accepted! Waiting for result." );
  std::shared_future<BabelFishActionClient::WrappedResult> result_future =
      client->async_get_result( goal_handle );
  BabelFishActionClient::WrappedResult result = result_future.get();
  auto &sequence = ( *result.result )["sequence"].as<ArrayMessage<int32_t>>();
  // Print result sequence
  std::cout << std::endl << "Result is an array of length:" << sequence.size() << std::endl;
  for ( size_t i = 0; i < sequence.size(); ++i ) {
    std::cout << sequence[i];
    if ( i != sequence.size() - 1 )
      std::cout << ", ";
  }
  std::cout << std::endl;
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
