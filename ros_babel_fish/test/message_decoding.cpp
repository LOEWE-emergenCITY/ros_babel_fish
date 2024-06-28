//
// Created by Stefan Fabian on 03.09.19.
//

#include "common.hpp"
#include "message_comparison.hpp"

#include <ros_babel_fish/babel_fish.hpp>
#include <ros_babel_fish_test_msgs/msg/test_array.hpp>
#include <ros_babel_fish_test_msgs/msg/test_message.hpp>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace ros_babel_fish;

std::shared_ptr<rclcpp::Node> node;

class MessageDecodingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std_msgs_header.frame_id = "babel_fish_frame";
    std_msgs_header.stamp = rclcpp::Time( 13, 37 );

    geometry_msgs_point.x = 1.23;
    geometry_msgs_point.y = 2.418;
    geometry_msgs_point.z = 4218;

    // Normalized quaternion because unnormalized quaternions should crash programs /s (Still sorry about that)
    geometry_msgs_quaternion.w = 0.123;
    geometry_msgs_quaternion.x = 0.4325;
    geometry_msgs_quaternion.y = 0.2011;
    geometry_msgs_quaternion.z = 0.2434;

    geometry_msgs_pose.position = geometry_msgs_point;
    geometry_msgs_pose.orientation = geometry_msgs_quaternion;

    geometry_msgs_pose_stamped.header = std_msgs_header;
    geometry_msgs_pose_stamped.pose = geometry_msgs_pose;

    test_message.header.stamp = rclcpp::Time( 42 );
    test_message.header.frame_id = "test";
    test_message.b = false;
    test_message.ui8 = 255;
    test_message.ui16 = 65535;
    test_message.ui32 = 4294967295;
    test_message.ui64 = std::numeric_limits<uint64_t>::max();
    test_message.i8 = -128;
    test_message.i16 = -32768;
    test_message.i32 = -2147483648;
    test_message.i64 = std::numeric_limits<int64_t>::min();
    test_message.f32 = 42.0;
    test_message.f64 = 1337.0;
    test_message.str = "test string";
    test_message.t = rclcpp::Time( 1337, 42 );
    test_message.d = rclcpp::Duration( 42, 1337 );
    for ( int i = 0; i < 5; ++i ) {
      geometry_msgs::msg::Point p;
      p.x = i * 0.1;
      p.y = 3 + i * 0.4;
      p.z = 15 + i * 3.14;
      test_message.point_arr.push_back( p );
    }

    unsigned SEED = 42;
    fillArray( test_array.bools, SEED++ );
    fillArray( test_array.uint8s, SEED++ );
    fillArray( test_array.uint16s, SEED++ );
    fillArray( test_array.uint32s, SEED++ );
    fillArray( test_array.uint64s, SEED++ );
    fillArray( test_array.int8s, SEED++ );
    fillArray( test_array.int16s, SEED++ );
    fillArray( test_array.int32s, SEED++ );
    fillArray( test_array.int64s, SEED++ );
    fillArray( test_array.float32s, SEED++ );
    fillArray( test_array.float64s, SEED++ );
    fillArray( test_array.times, SEED++ );
    fillArray( test_array.durations, SEED++ );
    fillArray( test_array.strings, SEED++ );
    fillArray( test_array.subarrays_fixed, SEED++ );
    fillArray( test_array.subarrays, SEED++ );

    header_pub_ = node->create_publisher<std_msgs::msg::Header>( "/test_message_decoding/header", 1 );
    point_pub_ =
        node->create_publisher<geometry_msgs::msg::Point>( "/test_message_decoding/point", 1 );
    quaternion_pub_ = node->create_publisher<geometry_msgs::msg::Quaternion>(
        "/test_message_decoding/quaternion", 1 );
    pose_pub_ = node->create_publisher<geometry_msgs::msg::Pose>( "/test_message_decoding/pose", 1 );
    pose_stamped_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/test_message_decoding/pose_stamped", 1 );
    test_message_pub_ = node->create_publisher<ros_babel_fish_test_msgs::msg::TestMessage>(
        "/test_message_decoding/test_message", 1 );
    test_sub_array_pub_ = node->create_publisher<ros_babel_fish_test_msgs::msg::TestSubArray>(
        "/test_message_decoding/sub_test_array", 1 );
    test_array_pub_ = node->create_publisher<ros_babel_fish_test_msgs::msg::TestArray>(
        "/test_message_decoding/test_array", 1 );

    using namespace std::chrono_literals;
    publish_timer_ = node->create_wall_timer( 50ms, [this] { publish(); } );
  }

  void publish()
  {
    header_pub_->publish( std_msgs_header );
    point_pub_->publish( geometry_msgs_point );
    quaternion_pub_->publish( geometry_msgs_quaternion );
    pose_pub_->publish( geometry_msgs_pose );
    pose_stamped_pub_->publish( geometry_msgs_pose_stamped );
    test_message_pub_->publish( test_message );
    test_sub_array_pub_->publish( test_array.subarrays[0] );
    test_array_pub_->publish( test_array );
  }

  BabelFish fish;
  std_msgs::msg::Header std_msgs_header;
  geometry_msgs::msg::Point geometry_msgs_point;
  geometry_msgs::msg::Quaternion geometry_msgs_quaternion;
  geometry_msgs::msg::Pose geometry_msgs_pose;
  geometry_msgs::msg::PoseStamped geometry_msgs_pose_stamped;
  ros_babel_fish_test_msgs::msg::TestMessage test_message;
  ros_babel_fish_test_msgs::msg::TestArray test_array;

  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr header_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr quaternion_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub_;
  rclcpp::Publisher<ros_babel_fish_test_msgs::msg::TestMessage>::SharedPtr test_message_pub_;
  rclcpp::Publisher<ros_babel_fish_test_msgs::msg::TestSubArray>::SharedPtr test_sub_array_pub_;
  rclcpp::Publisher<ros_babel_fish_test_msgs::msg::TestArray>::SharedPtr test_array_pub_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
};

TEST_F( MessageDecodingTest, tests )
{
  using namespace std::chrono_literals;
  rclcpp::GuardCondition::SharedPtr cond = std::make_shared<rclcpp::GuardCondition>();
  rclcpp::WaitSet set;
  set.add_guard_condition( cond );
  CompoundMessage::SharedPtr msg;
  std::function<void( CompoundMessage::SharedPtr )> callback(
      [&msg, &cond]( const CompoundMessage::SharedPtr &m ) {
        msg = m;
        cond->trigger();
      } );
  BabelFishSubscription::SharedPtr subscription =
      fish.create_subscription( *node, "/test_message_decoding/header", 1, callback );
  ASSERT_EQ( set.wait( 5s ).kind(), rclcpp::WaitResultKind::Ready );
  subscription.reset();

  ASSERT_EQ( msg->name(), "std_msgs/msg/Header" );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( std_msgs_header, *msg ) );

  // Actual copy can be created with clone
  {
    CompoundMessage copy = msg->clone();
    EXPECT_EQ( copy.name(), msg->name() );
    EXPECT_EQ( copy["frame_id"], ( *msg )["frame_id"] );
    copy["frame_id"] = "some_other_value";
    EXPECT_NE( copy["frame_id"], ( *msg )["frame_id"] );
    std::shared_ptr<void> data = copy.type_erased_message();
    EXPECT_NE( data, msg->type_erased_message() );
    copy = *msg;
    // Should still have separate but the same data pointer but now it should be equal to msg again.
    EXPECT_EQ( data, copy.type_erased_message() );
    EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( std_msgs_header, copy ) );
  }

  // Copy constructor should construct a copy pointing to the same message
  CompoundMessage copy = *msg;
  EXPECT_EQ( copy.name(), msg->name() );

  EXPECT_EQ( copy["frame_id"], ( *msg )["frame_id"] );
  copy["frame_id"] = "some_other_value";
  EXPECT_EQ( ( *msg )["frame_id"], "some_other_value" );

  subscription = fish.create_subscription( *node, "/test_message_decoding/point", 1, callback );
  ASSERT_EQ( set.wait( 5s ).kind(), rclcpp::WaitResultKind::Ready );
  subscription.reset();

  ASSERT_EQ( msg->name(), "geometry_msgs/msg/Point" );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( geometry_msgs_point, *msg ) );

  // Assignment should not work to different type
  EXPECT_THROW( copy = *msg, BabelFishException );
  // Move assignment should work
  copy = std::move( CompoundMessage( *msg ) );
  EXPECT_EQ( copy.name(), msg->name() );

  // Create subscription with known type
  subscription = fish.create_subscription( *node, "/test_message_decoding/quaternion",
                                           "geometry_msgs/msg/Quaternion", 1, callback );
  ASSERT_EQ( set.wait( 5s ).kind(), rclcpp::WaitResultKind::Ready );
  subscription.reset();

  ASSERT_EQ( msg->name(), "geometry_msgs/msg/Quaternion" );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( geometry_msgs_quaternion, *msg ) );
  EXPECT_EQ( ( *msg )["x"], geometry_msgs_quaternion.x );

  subscription = fish.create_subscription( *node, "/test_message_decoding/pose", 1, callback );
  ASSERT_EQ( set.wait( 5s ).kind(), rclcpp::WaitResultKind::Ready );
  subscription.reset();

  EXPECT_EQ( msg->name(), "geometry_msgs/msg/Pose" );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( geometry_msgs_pose, *msg ) );

  subscription =
      fish.create_subscription( *node, "/test_message_decoding/pose_stamped", 1, callback );
  ASSERT_EQ( set.wait( 5s ).kind(), rclcpp::WaitResultKind::Ready );
  subscription.reset();

  EXPECT_EQ( msg->name(), "geometry_msgs/msg/PoseStamped" );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( geometry_msgs_pose_stamped, *msg ) );

  subscription =
      fish.create_subscription( *node, "/test_message_decoding/test_message", 1, callback );
  ASSERT_EQ( set.wait( 5s ).kind(), rclcpp::WaitResultKind::Ready );
  subscription.reset();

  ASSERT_EQ( msg->name(), "ros_babel_fish_test_msgs/msg/TestMessage" );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( test_message, *msg ) );
}

TEST_F( MessageDecodingTest, arrayTests )
{
  rclcpp::GuardCondition::SharedPtr cond = std::make_shared<rclcpp::GuardCondition>();
  rclcpp::WaitSet set;
  set.add_guard_condition( cond );
  CompoundMessage::SharedPtr msg;
  std::function<void( CompoundMessage::SharedPtr )> callback(
      [&msg, &cond]( const CompoundMessage::SharedPtr &m ) {
        msg = m;
        cond->trigger();
      } );
  BabelFishSubscription::SharedPtr subscription =
      fish.create_subscription( *node, "/test_message_decoding/sub_test_array", 1, callback );
  ASSERT_EQ( set.wait().kind(), rclcpp::WaitResultKind::Ready );
  subscription.reset();

  ASSERT_EQ( msg->name(), "ros_babel_fish_test_msgs/msg/TestSubArray" );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( test_array.subarrays[0], *msg ) );

  subscription = fish.create_subscription( *node, "/test_message_decoding/test_array", 1, callback );
  ASSERT_EQ( set.wait().kind(), rclcpp::WaitResultKind::Ready );
  subscription.reset();

  ASSERT_EQ( msg->name(), "ros_babel_fish_test_msgs/msg/TestArray" );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( test_array, *msg ) );
  FixedLengthCompoundArrayMessage msg_subarrays_fixed =
      ( *msg )["subarrays_fixed"].as<FixedLengthCompoundArrayMessage>();
  ASSERT_EQ( msg_subarrays_fixed.elementDatatype(), "ros_babel_fish_test_msgs::msg::TestSubArray" );
  ASSERT_EQ( msg_subarrays_fixed.elementName(), "ros_babel_fish_test_msgs/msg/TestSubArray" );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  node = std::make_shared<rclcpp::Node>( "message_decoding_test" );
  std::thread spinner( []() { rclcpp::spin( node ); } );
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  spinner.join();
  node.reset();
  return result;
}
