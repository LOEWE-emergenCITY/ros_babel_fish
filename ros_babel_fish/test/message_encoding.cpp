//
// Created by Stefan Fabian on 04.09.19.
//

#include "message_comparison.hpp"

#include <ros_babel_fish/babel_fish.hpp>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <random>

using namespace ros_babel_fish;

std::shared_ptr<rclcpp::Node> node;

template<typename Message>
std::shared_ptr<Message> waitForMessage( const std::string &topic )
{
  rclcpp::QoS qos = rclcpp::QoS( 1 ).transient_local();
  auto sub = node->create_subscription<Message>( topic, qos, []( std::unique_ptr<Message> ) {} );
  rclcpp::ThreadSafeWaitSet wait_set(
      std::vector<rclcpp::ThreadSafeWaitSet::SubscriptionEntry>{ { sub } } );
  auto wait_result = wait_set.template wait();
  if ( wait_result.kind() != rclcpp::WaitResultKind::Ready )
    return nullptr;
  std::shared_ptr<Message> result = std::make_shared<Message>();
  rclcpp::MessageInfo info;
  if ( !sub->take( *result, info ) )
    return nullptr;
  return result;
}

template<typename T, bool BOUNDED = false, bool FIXED_LENGTH = false>
typename std::enable_if<!std::is_same<T, bool>::value and !std::is_same<T, std::string>::value, void>::type
fillArray( ArrayMessage_<T, BOUNDED, FIXED_LENGTH> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  typedef
      typename std::conditional<std::is_floating_point<T>::value, std::uniform_real_distribution<T>,
                                std::uniform_int_distribution<T>>::type Distribution;
  Distribution distribution( std::numeric_limits<T>::min(), std::numeric_limits<T>::max() );
  if ( msg.isFixedSize() ) {
    for ( size_t i = 0; i < msg.size(); ++i ) { msg.assign( i, distribution( generator ) ); }
    return;
  }
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.resize( length );
  for ( size_t i = 0; i < length; ++i ) { msg.at( i ) = distribution( generator ); }
}

template<typename T, bool BOUNDED = false, bool FIXED_LENGTH = false>
typename std::enable_if<std::is_same<T, bool>::value, void>::type
fillArray( ArrayMessage_<T, BOUNDED, FIXED_LENGTH> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_int_distribution<uint8_t> distribution( 0, 1 );
  if ( msg.isFixedSize() ) {
    for ( size_t i = 0; i < msg.size(); ++i ) { msg.assign( i, distribution( generator ) == 1 ); }
    return;
  }
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.resize( length );
  for ( size_t i = 0; i < length; ++i ) { msg.at( i ) = distribution( generator ) == 1; }
}

std::string randomString( unsigned seed, int length = -1 )
{
  std::default_random_engine generator( seed );
  static const char alphanum[] = "0123456789"
                                 "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                 "abcdefghijklmnopqrstuvwxyz";
  std::uniform_int_distribution<size_t> distribution( 0, sizeof( alphanum ) - 2 );
  if ( length == -1 ) {
    std::uniform_int_distribution<int> length_distribution( 1, 1000 );
    length = length_distribution( generator );
  }
  std::vector<char> result( length + 1 );
  for ( int i = 0; i < length; ++i ) { result[i] = alphanum[distribution( generator )]; }
  result[length] = 0;
  return std::string( result.data() );
}

template<typename T, bool BOUNDED = false, bool FIXED_LENGTH = false>
typename std::enable_if<std::is_same<T, std::string>::value, void>::type
fillArray( ArrayMessage_<T, BOUNDED, FIXED_LENGTH> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_int_distribution<unsigned> distribution( std::numeric_limits<unsigned>::min() );
  if ( msg.isFixedSize() ) {
    for ( size_t i = 0; i < msg.size(); ++i ) {
      msg.assign( i, randomString( distribution( generator ), i == 0 ? 1 : -1 ) );
    }
    return;
  }
  std::uniform_int_distribution<size_t> length_distribution( BOUNDED ? 1 : 10,
                                                             BOUNDED ? msg.maxSize() : 1000 );
  size_t length = length_distribution( generator );
  msg.resize( length );
  for ( size_t i = 0; i < length; ++i ) {
    msg.at( i ) = randomString( distribution( generator ), i == 0 ? 1 : -1 );
  }
}

template<typename T, bool BOUNDED = false, bool FIXED_LENGTH = false>
typename std::enable_if<std::is_same<T, rclcpp::Time>::value, void>::type
fillArray( CompoundArrayMessage_<BOUNDED, FIXED_LENGTH> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_real_distribution<double> distribution( 0, 1E9 );
  if ( msg.isFixedSize() ) {
    for ( size_t i = 0; i < msg.size(); ++i ) {
      msg[i] = rclcpp::Time( distribution( generator ) );
    }
    return;
  }
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.resize( length );
  for ( size_t i = 0; i < length; ++i ) { msg.at( i ) = rclcpp::Time( distribution( generator ) ); }
}

template<typename T, bool BOUNDED = false, bool FIXED_LENGTH = false>
typename std::enable_if<std::is_same<T, rclcpp::Duration>::value, void>::type
fillArray( CompoundArrayMessage_<BOUNDED, FIXED_LENGTH> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_int_distribution<long> distribution( -1E9, 1E9 );
  if ( msg.isFixedSize() ) {
    for ( size_t i = 0; i < msg.size(); ++i ) {
      msg[i] = rclcpp::Duration( std::chrono::nanoseconds( distribution( generator ) ) );
    }
    return;
  }
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.resize( length );
  for ( size_t i = 0; i < length; ++i ) {
    // int64_t nanoseconds constructor deprecated in galactic but from_nanoseconds not available in foxy
#if RCLCPP_VERSION_MAJOR >= 9
    msg.at( i ) = rclcpp::Duration::from_nanoseconds( distribution( generator ) );
#else
    msg.at( i ) = rclcpp::Duration( distribution( generator ) );
#endif
  }
}

template<bool BOUNDED = false, bool FIXED_LENGTH = false>
void fillArray( CompoundArrayMessage_<BOUNDED, FIXED_LENGTH> &msg, unsigned seed )
{
  std::default_random_engine generator( seed );
  std::uniform_int_distribution<unsigned> distribution( std::numeric_limits<unsigned>::min() );
  if ( msg.isFixedSize() ) {
    for ( size_t i = 0; i < msg.size(); ++i ) {
      fillArray( msg[i]["ints"].template as<ArrayMessage<int32_t>>(), seed++ );
      fillArray( msg[i]["strings"].template as<BoundedArrayMessage<std::string>>(), seed++ );
      fillArray<rclcpp::Time>( msg[i]["times"].template as<FixedLengthCompoundArrayMessage>(),
                               seed++ );
    }
    return;
  }
  std::uniform_int_distribution<size_t> length_distribution( 10, 1000 );
  size_t length = length_distribution( generator );
  msg.resize( length );
  for ( size_t i = 0; i < length; ++i ) {
    fillArray( msg[i]["ints"].template as<ArrayMessage<int32_t>>(), seed++ );
    fillArray( msg[i]["strings"].template as<BoundedArrayMessage<std::string>>(), seed++ );
    fillArray<rclcpp::Time>( msg[i]["times"].template as<FixedLengthCompoundArrayMessage>(), seed++ );
  }
}

class MessageEncodingTest : public ::testing::Test
{
protected:
  void publish( const std::string &topic, const CompoundMessage::SharedPtr &msg )
  {
    BabelFishPublisher::SharedPtr pub =
        fish->create_publisher( *node, topic, msg->name(), rclcpp::QoS( 1 ).transient_local() );
    pub->publish( *msg );
    publishers_.push_back( pub );
  }

  void publish( const std::string &topic, const CompoundMessage &msg )
  {
    BabelFishPublisher::SharedPtr pub =
        fish->create_publisher( *node, topic, msg.name(), rclcpp::QoS( 1 ).transient_local() );
    pub->publish( msg );
    publishers_.push_back( pub );
  }

  void SetUp() override
  {
    fish = BabelFish::make_shared();
    std_msgs_header = fish->create_message_shared( "std_msgs/Header" );
    {
      CompoundMessage compound = *std_msgs_header;
      compound["frame_id"] = "babel_fish_frame";
      compound["stamp"] = rclcpp::Time( 13.37 );
    }

    geometry_msgs_point = fish->create_message_shared( "geometry_msgs/Point" );
    {
      CompoundMessage compound = *geometry_msgs_point;
      compound["x"] = 1.23;
      compound["y"] = 2.418;
      compound["z"] = 4218;
    }

    // Normalized quaternion because unnormalized quaternions should crash programs /s (Still sorry about that)
    geometry_msgs_quaternion = fish->create_message_shared( "geometry_msgs/Quaternion" );
    {
      CompoundMessage compound = *geometry_msgs_quaternion;
      compound["w"] = 0.6325;
      compound["x"] = 0.123;
      compound["y"] = 0.1434;
      compound["z"] = 0.1011;
    }

    geometry_msgs_pose = fish->create_message_shared( "geometry_msgs/Pose" );
    {
      CompoundMessage compound = *geometry_msgs_pose;
      compound["position"] = *geometry_msgs_point;
      compound["orientation"] = *geometry_msgs_quaternion;
    }

    geometry_msgs_pose_stamped = fish->create_message_shared( "geometry_msgs/PoseStamped" );
    {
      CompoundMessage compound = *geometry_msgs_pose_stamped;
      compound["header"] = *std_msgs_header;
      compound["pose"] = *geometry_msgs_pose;
    }

    test_msg = fish->create_message_shared( "ros_babel_fish_test_msgs/TestMessage" );
    {
      CompoundMessage compound = *test_msg;
      compound["header"]["stamp"] = rclcpp::Time( 42 );
      compound["header"]["frame_id"] = "test";
      compound["b"] = false;
      compound["ui8"] = 255;
      compound["ui16"] = 65535;
      compound["ui32"] = 4294967295;
      compound["ui64"] = std::numeric_limits<uint64_t>::max();
      compound["i8"] = -128;
      compound["i16"] = -32768;
      compound["i32"] = -2147483648;
      compound["i64"] = std::numeric_limits<int64_t>::min();
      compound["f32"] = 42.0f;
      compound["f64"] = 1337.0;
      compound["str"] = "test string";
      compound["t"] = rclcpp::Time( 1337, 42 );
      compound["d"] = rclcpp::Duration( 42, 1337 );
      auto &cam = compound["point_arr"].as<CompoundArrayMessage>();
      cam.resize( 5 );
      for ( int i = 0; i < 5; ++i ) {
        auto &pose = cam.at( i );
        pose["x"] = i * 0.1;
        pose["y"] = 3 + i * 0.4;
        pose["z"] = 15 + i * 3.14;
      }
    }

    test_array_msg = fish->create_message_shared( "ros_babel_fish_test_msgs/TestArray" );
    {
      CompoundMessage compound = *test_array_msg;
      unsigned SEED = 42;
      fillArray( compound["bools"].as<ArrayMessage<bool>>(), SEED++ );
      fillArray( compound["uint8s"].as<ArrayMessage<uint8_t>>(), SEED++ );
      fillArray( compound["uint16s"].as<FixedLengthArrayMessage<uint16_t>>(), SEED++ );
      fillArray( compound["uint32s"].as<ArrayMessage<uint32_t>>(), SEED++ );
      fillArray( compound["uint64s"].as<ArrayMessage<uint64_t>>(), SEED++ );
      fillArray( compound["int8s"].as<ArrayMessage<int8_t>>(), SEED++ );
      fillArray( compound["int16s"].as<ArrayMessage<int16_t>>(), SEED++ );
      fillArray( compound["int32s"].as<ArrayMessage<int32_t>>(), SEED++ );
      fillArray( compound["int64s"].as<FixedLengthArrayMessage<int64_t>>(), SEED++ );
      fillArray( compound["float32s"].as<ArrayMessage<float>>(), SEED++ );
      fillArray( compound["float64s"].as<BoundedArrayMessage<double>>(), SEED++ );
      fillArray<rclcpp::Time>( compound["times"].as<CompoundArrayMessage>(), SEED++ );
      fillArray<rclcpp::Duration>( compound["durations"].as<FixedLengthCompoundArrayMessage>(),
                                   SEED++ );
      fillArray( compound["strings"].as<ArrayMessage<std::string>>(), SEED++ );
      fillArray( compound["subarrays_fixed"].as<FixedLengthCompoundArrayMessage>(), SEED++ );
      fillArray( compound["subarrays"].as<CompoundArrayMessage>(), SEED++ );
    }

    // Publish
    publish( "/test_message_encoding/header", std_msgs_header );
    publish( "/test_message_encoding/point", geometry_msgs_point );
    publish( "/test_message_encoding/quaternion", geometry_msgs_quaternion );
    publish( "/test_message_encoding/pose", geometry_msgs_pose );
    publish( "/test_message_encoding/pose_stamped", geometry_msgs_pose_stamped );
    publish( "/test_message_encoding/test_message", test_msg );
    publish( "/test_message_encoding/test_array", test_array_msg );
    CompoundMessage::SharedPtr sub_test_array =
        fish->create_message_shared( "ros_babel_fish_test_msgs/msg/TestSubArray" );
    *sub_test_array = ( *test_array_msg )["subarrays"].as<CompoundArrayMessage>()[0];
    publish( "/test_message_encoding/sub_test_array", sub_test_array );
  }

  std::vector<BabelFishPublisher::SharedPtr> publishers_;

  BabelFish::SharedPtr fish;

  CompoundMessage::SharedPtr std_msgs_header;
  CompoundMessage::SharedPtr geometry_msgs_point;
  CompoundMessage::SharedPtr geometry_msgs_quaternion;
  CompoundMessage::SharedPtr geometry_msgs_pose;
  CompoundMessage::SharedPtr geometry_msgs_pose_stamped;
  CompoundMessage::SharedPtr test_array_msg;
  CompoundMessage::SharedPtr test_msg;
};

TEST_F( MessageEncodingTest, tests )
{
  auto msg_header = waitForMessage<std_msgs::msg::Header>( "/test_message_encoding/header" );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( *std_msgs_header, msg_header ) );

  auto msg_point = waitForMessage<geometry_msgs::msg::Point>( "/test_message_encoding/point" );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( *geometry_msgs_point, msg_point ) );

  auto msg_quaternion =
      waitForMessage<geometry_msgs::msg::Quaternion>( "/test_message_encoding/quaternion" );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( *geometry_msgs_quaternion, msg_quaternion ) );

  auto msg_pose = waitForMessage<geometry_msgs::msg::Pose>( "/test_message_encoding/pose" );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( *geometry_msgs_pose, msg_pose ) );

  auto msg_pose_stamped =
      waitForMessage<geometry_msgs::msg::PoseStamped>( "/test_message_encoding/pose_stamped" );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( *geometry_msgs_pose_stamped, msg_pose_stamped ) );
  geometry_msgs::msg::PoseStamped pose_reference;
  pose_reference.header.stamp = rclcpp::Time( 13.37 );
  pose_reference.header.frame_id = "babel_fish_frame";
  pose_reference.pose.position.x = 1.23;
  pose_reference.pose.position.y = 1.23;
  pose_reference.pose.position.z = 1.23;
  pose_reference.pose.orientation.w = 1.23;

  auto msg_test_message = waitForMessage<ros_babel_fish_test_msgs::msg::TestMessage>(
      "/test_message_encoding/test_message" );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( *test_msg, msg_test_message ) );
}

TEST_F( MessageEncodingTest, arrayTests )
{
  auto msg_header = waitForMessage<std_msgs::msg::Header>( "/test_message_encoding/header" );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( *std_msgs_header, msg_header ) );

  auto msg_sub_test_array = waitForMessage<ros_babel_fish_test_msgs::msg::TestSubArray>(
      "/test_message_encoding/sub_test_array" );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( ( *test_array_msg )["subarrays"].as<CompoundArrayMessage>()[0],
                                      msg_sub_test_array ) );

  auto msg_test_array = waitForMessage<ros_babel_fish_test_msgs::msg::TestArray>(
      "/test_message_encoding/test_array" );
  EXPECT_TRUE( MESSAGE_CONTENT_EQUAL( *test_array_msg, msg_test_array ) );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  node = std::make_shared<rclcpp::Node>( "test_message_decoding" );
  int result = RUN_ALL_TESTS();
  node.reset();
  return result;
}
