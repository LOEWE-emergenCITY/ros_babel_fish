//
// Created by Stefan Fabian on 03.03.20.
//

#include "message_comparison.hpp"
#include <ros_babel_fish/babel_fish.hpp>

#include <action_msgs/msg/goal_status_array.h>
#include <action_tutorials_interfaces/action/fibonacci.hpp>
#include <ros_babel_fish_test_msgs/action/simple_test.hpp>

#include <rclcpp/rclcpp.hpp>
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
  //  EXPECT_EQ(type_support->type_support_handle, *ts);
  EXPECT_NE( ts, nullptr );
  EXPECT_TRUE( Equal( type_support->type_support_handle, *ts ) );

  type_support = fish.get_action_type_support( "action_tutorials_interfaces/action/Fibonacci" );
  ASSERT_NE( type_support, nullptr );

  const auto *ts_map =
      static_cast<const type_support_map_t *>( ts->status_message_type_support->data );
  const auto *map = static_cast<const type_support_map_t *>(
      type_support->status_message_type_support->type_support_handle.data );
  EXPECT_EQ( ts_map, map );
}

TEST( ActionClientTest, simpleActionClient )
{
  //  auto provider = std::make_shared<MessageOnlyDescriptionProvider>();
  //  BabelFish fish( provider );
  //  provider->registerMessageByDefinition( ros::message_traits::datatype<SimpleTestAction>(),
  //                                         ros::message_traits::definition<SimpleTestAction>());
  //  MessageDescription::ConstSharedPtr goal_description = provider->getMessageDescription(
  //    ros::message_traits::datatype<SimpleTestActionGoal>());
  //  actionlib::SimpleActionClient <BabelFishAction> client( goal_description, "simple" );
  //  if ( !client.waitForServer( ros::Duration( 10 )))
  //    FAIL() << "ActionServer did not start within 10 seconds!";
  //  ASSERT_TRUE( client.isServerConnected());
  //
  //  // This goal should succeed
  //  Message::SharedPtr goal = fish.createMessage( "ros_babel_fish_test_msgs/SimpleTestGoal" );
  //  (*goal)["goal"] = 5;
  //  CompoundMessage::ConstSharedPtr goal_msg = fish.translateMessage( goal );
  //  actionlib::SimpleClientGoalState state = client.sendGoalAndWait( *goal_msg, ros::Duration( 10 ));
  //  EXPECT_EQ( state, actionlib::SimpleClientGoalState::SUCCEEDED );
  //  CompoundMessage::ConstSharedPtr result = client.getResult();
  //  TranslatedMessage::ConstSharedPtr translated = fish.translateMessage( result );
  //  EXPECT_EQ((*translated->translated_message)["result"].value<int32_t>(), 4 );
  //
  //  // This goal should abort after 10
  //  goal = fish.createMessage( "ros_babel_fish_test_msgs/SimpleTestGoal" );
  //  (*goal)["goal"] = 20;
  //  goal_msg = fish.translateMessage( goal );
  //  std::vector<int> feedback_values;
  //  client.sendGoal( *goal_msg, {}, {}, boost::function < void(
  //  const CompoundMessage::ConstSharedPtr & )>(
  //    [ & ]( const CompoundMessage::ConstSharedPtr &feedback )
  //    {
  //      TranslatedMessage::ConstSharedPtr translated = fish.translateMessage( feedback );
  //      feedback_values.push_back((*translated->translated_message)["feedback"].value<int32_t>());
  //    }));
  //  ASSERT_EQ( client.getState(), actionlib::SimpleClientGoalState::PENDING );
  //  if ( !client.waitForResult( ros::Duration( 10 )))
  //  {
  //    FAIL() << "ActionServer did not finish in 10 seconds!";
  //  }
  //  ASSERT_EQ( feedback_values.size(), 10U );
  //  for ( int i = 0; i < 10; ++i )
  //  {
  //    if ( feedback_values[i] != i ) FAIL() << "Feedback at " << i << " should be " << i << "!";
  //  }
  //  EXPECT_EQ( client.getState(), actionlib::SimpleClientGoalState::ABORTED );
  //  result = client.getResult();
  //  translated = fish.translateMessage( result );
  //  EXPECT_EQ((*translated->translated_message)["result"].value<int32_t>(), 10 );
  //
  //  // This goal should be preempted
  //  goal = fish.createMessage( "ros_babel_fish_test_msgs/SimpleTestGoal" );
  //  (*goal)["goal"] = 1000;
  //  goal_msg = fish.translateMessage( goal );
  //  feedback_values.clear();
  //  client.sendGoal( *goal_msg, {}, {}, boost::function < void(
  //  const CompoundMessage::ConstSharedPtr & )>(
  //    [ & ]( const CompoundMessage::ConstSharedPtr &feedback )
  //    {
  //      TranslatedMessage::ConstSharedPtr translated = fish.translateMessage( feedback );
  //      feedback_values.push_back((*translated->translated_message)["feedback"].value<int32_t>());
  //    }));
  //  usleep( 500000 ); // Sleep for 500ms
  //  client.cancelGoal();
  //  if ( !client.waitForResult( ros::Duration( 1 )))
  //    FAIL() << "ActionServer did not preempt in 1 second!";
  //  int last_feedback = 0;
  //  for ( size_t i = 0; i < feedback_values.size(); ++i )
  //  {
  //    if ( feedback_values[i] != int( i )) FAIL() << "Feedback at " << i << " should be " << i << "!";
  //    last_feedback = feedback_values[i];
  //  }
  //  EXPECT_EQ( client.getState(), actionlib::SimpleClientGoalState::PREEMPTED );
  //  result = client.getResult();
  //  translated = fish.translateMessage( result );
  //  EXPECT_EQ((*translated->translated_message)["result"].value<int32_t>(), last_feedback );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  node = std::make_shared<rclcpp::Node>( "service_client_test" );
  std::thread spinner( []() { rclcpp::spin( node ); } );
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  spinner.join();
  node.reset();
  return result;
}
