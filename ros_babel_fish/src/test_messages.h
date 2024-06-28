//
// Created by Stefan Fabian on 25.05.21.
//

#ifndef ROS_BABEL_FISH_TEST_MESSAGES_H
#define ROS_BABEL_FISH_TEST_MESSAGES_H

#include "logging.hpp"
#include "ros_babel_fish/babel_fish.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ros_babel_fish_test_msgs/msg/test_message.hpp>

namespace ros_babel_fish
{

template<typename MessageType>
void initMessage( MessageType &msg );

template<typename MessageType>
bool isSameDatatype( const MessageType &, const ros_babel_fish::CompoundMessage &fish_msg )
{
  if ( rosidl_generator_traits::data_type<MessageType>() != fish_msg.datatype() ) {
    RBF2_ERROR_STREAM( "Name of received message does not match!"
                       << std::endl
                       << "Expected: " << rosidl_generator_traits::name<MessageType>() << std::endl
                       << "Received: " << fish_msg.datatype() );
    return false;
  }
  return true;
}

template<typename T>
bool fail( const std::string &field, const T &expected, const T &actual )
{
  RBF2_ERROR_STREAM( "Content of received message does not match at field: "
                     << field << "!" << std::endl
                     << "Expected: " << expected << "Received: " << actual );
  return false;
}

template<>
bool fail( const std::string &field, const rclcpp::Time &expected, const rclcpp::Time &actual )
{
  RBF2_ERROR_STREAM( "Content of received message does not match at field: "
                     << field << "!" << std::endl
                     << "Expected: " << expected.seconds() << "Received: " << actual.seconds() );
  return false;
}

template<>
bool fail( const std::string &field, const rclcpp::Duration &expected, const rclcpp::Duration &actual )
{
  RBF2_ERROR_STREAM( "Content of received message does not match at field: "
                     << field << "!" << std::endl
                     << "Expected: " << expected.seconds() << "Received: " << actual.seconds() );
  return false;
}

template<typename MessageType>
bool messagesAreEqual( const MessageType &msg, const CompoundMessage &translated );

#define CHECK_MEMBER_EQUAL( NAME, TYPE )                                                           \
  if ( TYPE( msg.NAME ) != translated[#NAME].value<TYPE>() )                                       \
  return fail( #NAME, TYPE( msg.NAME ), translated[#NAME].value<TYPE>() )

template<>
bool messagesAreEqual( const std_msgs::msg::Header &msg, const CompoundMessage &translated )
{
  CHECK_MEMBER_EQUAL( frame_id, std::string );
  CHECK_MEMBER_EQUAL( stamp, rclcpp::Time );
  return true;
}

template<>
bool messagesAreEqual( const geometry_msgs::msg::Point &msg, const CompoundMessage &translated )
{
  CHECK_MEMBER_EQUAL( x, double );
  CHECK_MEMBER_EQUAL( y, double );
  CHECK_MEMBER_EQUAL( z, double );
  return true;
}

template<>
bool messagesAreEqual( const geometry_msgs::msg::Quaternion &msg, const CompoundMessage &translated )
{
  CHECK_MEMBER_EQUAL( w, double );
  CHECK_MEMBER_EQUAL( x, double );
  CHECK_MEMBER_EQUAL( y, double );
  CHECK_MEMBER_EQUAL( z, double );
  return true;
}

template<>
bool messagesAreEqual( const geometry_msgs::msg::Pose &msg, const CompoundMessage &translated )
{
  if ( !messagesAreEqual( msg.position, translated["position"].as<CompoundMessage>() ) )
    return false;
  if ( !messagesAreEqual( msg.orientation, translated["orientation"].as<CompoundMessage>() ) )
    return false;
  return true;
}

template<>
bool messagesAreEqual( const geometry_msgs::msg::PoseStamped &msg, const CompoundMessage &translated )
{
  if ( !messagesAreEqual( msg.header, translated["header"].as<CompoundMessage>() ) )
    return false;
  if ( !messagesAreEqual( msg.pose, translated["pose"].as<CompoundMessage>() ) )
    return false;
  return true;
}

template<>
bool messagesAreEqual( const ros_babel_fish_test_msgs::msg::TestMessage &msg,
                       const CompoundMessage &translated )
{
  if ( !messagesAreEqual( msg.header, translated["header"].as<CompoundMessage>() ) )
    return false;
  CHECK_MEMBER_EQUAL( b, bool );
  CHECK_MEMBER_EQUAL( f32, float );
  CHECK_MEMBER_EQUAL( f64, double );
  CHECK_MEMBER_EQUAL( i8, int8_t );
  CHECK_MEMBER_EQUAL( i16, int16_t );
  CHECK_MEMBER_EQUAL( i32, int32_t );
  CHECK_MEMBER_EQUAL( i64, int64_t );
  CHECK_MEMBER_EQUAL( ui8, uint8_t );
  CHECK_MEMBER_EQUAL( ui16, uint16_t );
  CHECK_MEMBER_EQUAL( ui32, uint32_t );
  CHECK_MEMBER_EQUAL( ui64, uint64_t );
  CHECK_MEMBER_EQUAL( d, rclcpp::Duration );
  CHECK_MEMBER_EQUAL( t, rclcpp::Time );
  CHECK_MEMBER_EQUAL( str, std::string );
  return true;
}

template<>
void initMessage( std_msgs::msg::Header &msg )
{
  msg.frame_id = "test_frame";
  msg.stamp = rclcpp::Time( 1337 );
}

template<>
void initMessage( geometry_msgs::msg::PoseStamped &msg )
{
  initMessage( msg.header );
  msg.pose.position.x = 1.1;
  msg.pose.position.y = 1.3;
  msg.pose.position.z = 0.5;
  msg.pose.orientation.w = 0.707;
  msg.pose.orientation.y = 0.707;
}

template<>
void initMessage( ros_babel_fish_test_msgs::msg::TestMessage &msg )
{
  using namespace std::chrono_literals;
  initMessage( msg.header );
  msg.b = true;
  msg.ui8 = 129;
  msg.ui16 = 45192;
  msg.ui32 = 4204967296;
  msg.ui64 = 4294967295UL * 4294967296UL;
  msg.i8 = -128;
  msg.i16 = 30000;
  msg.i32 = 1250132132;
  msg.i64 = -1234567890101112131;
  msg.f32 = 3.1415;
  msg.f64 = 2.718281828;
  msg.str = "test_string";
  msg.d = rclcpp::Duration( 42ns );
  for ( size_t i = 0; i < 3; ++i ) {
    geometry_msgs::msg::Point point;
    point.x = static_cast<double>( i ) / 0.3;
    point.y = static_cast<double>( i ) / 0.4;
    point.z = static_cast<double>( i ) / 0.15;
    msg.point_arr.push_back( point );
  }
}
} // namespace ros_babel_fish

#endif // ROS_BABEL_FISH_TEST_MESSAGES_H
