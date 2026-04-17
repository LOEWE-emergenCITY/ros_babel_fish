// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <ros_babel_fish/babel_fish.hpp>
#include <ros_babel_fish_tools/yaml_cpp_serialization.hpp>

#include <gtest/gtest.h>

using namespace ros_babel_fish;
using namespace ros_babel_fish_tools;

class YamlSerializationTest : public ::testing::Test
{
protected:
  BabelFish fish;
};

TEST_F( YamlSerializationTest, basicTypes )
{
  CompoundMessage msg = fish.create_message( "ros_babel_fish_test_msgs/msg/TestMessage" );
  msg["b"] = true;
  msg["ui8"] = (uint8_t)42;
  msg["ui16"] = (uint16_t)1234;
  msg["ui32"] = (uint32_t)123456;
  msg["ui64"] = (uint64_t)123456789;
  msg["i8"] = (int8_t)-42;
  msg["i16"] = (int16_t)-1234;
  msg["i32"] = (int32_t)-123456;
  msg["i64"] = (int64_t)-123456789;
  msg["f32"] = 1.23f;
  msg["f64"] = 4.56;
  msg["str"] = "Hello World";

  YAML::Node node;
  ASSERT_NO_THROW( node = message_to_yaml( msg ) );
  EXPECT_TRUE( node.IsMap() );
  EXPECT_EQ( node["b"].as<bool>(), true );
  EXPECT_EQ( node["ui8"].as<uint16_t>(), 42 );
  EXPECT_EQ( node["ui16"].as<uint16_t>(), 1234 );
  EXPECT_EQ( node["ui32"].as<uint32_t>(), 123456 );
  EXPECT_EQ( node["ui64"].as<uint64_t>(), 123456789 );
  EXPECT_EQ( node["i8"].as<int16_t>(), -42 );
  EXPECT_EQ( node["i16"].as<int16_t>(), -1234 );
  EXPECT_EQ( node["i32"].as<int32_t>(), -123456 );
  EXPECT_EQ( node["i64"].as<int64_t>(), -123456789 );
  EXPECT_NEAR( node["f32"].as<float>(), 1.23f, 1e-6 );
  EXPECT_NEAR( node["f64"].as<double>(), 4.56, 1e-6 );
  EXPECT_EQ( node["str"].as<std::string>(), "Hello World" );

  CompoundMessage msg2 = fish.create_message( "ros_babel_fish_test_msgs/msg/TestMessage" );
  ASSERT_NO_THROW( yaml_to_message( node, msg2 ) );
  EXPECT_EQ( msg2["b"].value<bool>(), true );
  EXPECT_EQ( msg2["ui8"].value<uint8_t>(), 42 );
  EXPECT_EQ( msg2["ui16"].value<uint16_t>(), 1234 );
  EXPECT_EQ( msg2["ui32"].value<uint32_t>(), 123456 );
  EXPECT_EQ( msg2["ui64"].value<uint64_t>(), 123456789 );
  EXPECT_EQ( msg2["i8"].value<int8_t>(), -42 );
  EXPECT_EQ( msg2["i16"].value<int16_t>(), -1234 );
  EXPECT_EQ( msg2["i32"].value<int32_t>(), -123456 );
  EXPECT_EQ( msg2["i64"].value<int64_t>(), -123456789 );
  EXPECT_NEAR( msg2["f32"].value<float>(), 1.23f, 1e-6 );
  EXPECT_NEAR( msg2["f64"].value<double>(), 4.56, 1e-6 );
  EXPECT_EQ( msg2["str"].value<std::string>(), "Hello World" );
}

TEST_F( YamlSerializationTest, arrays )
{
  CompoundMessage msg = fish.create_message( "ros_babel_fish_test_msgs/msg/TestArray" );
  auto &uint8_array = msg["uint8s"].as<ArrayMessage<uint8_t>>();
  uint8_array.resize( 3 );
  uint8_array.assign( 0, 1 );
  uint8_array.assign( 1, 2 );
  uint8_array.assign( 2, 3 );

  YAML::Node node;
  ASSERT_NO_THROW( node = message_to_yaml( msg ) ) << "Failed at message_to_yaml";
  EXPECT_TRUE( node["uint8s"].IsSequence() );
  EXPECT_EQ( node["uint8s"][0].as<uint16_t>(), 1 );
  EXPECT_EQ( node["uint8s"][1].as<uint16_t>(), 2 );
  EXPECT_EQ( node["uint8s"][2].as<uint16_t>(), 3 );

  CompoundMessage msg2 = fish.create_message( "ros_babel_fish_test_msgs/msg/TestArray" );
  ASSERT_NO_THROW( yaml_to_message( node, msg2 ) ) << "Failed at yaml_to_message";
  auto &uint8_array2 = msg2["uint8s"].as<ArrayMessage<uint8_t>>();
  EXPECT_EQ( uint8_array2.size(), 3u );
  EXPECT_EQ( uint8_array2[0], 1 );
  EXPECT_EQ( uint8_array2[1], 2 );
  EXPECT_EQ( uint8_array2[2], 3 );
}

TEST_F( YamlSerializationTest, timeAndDuration )
{
  CompoundMessage msg = fish.create_message( "ros_babel_fish_test_msgs/msg/TestMessage" );
  msg["t"]["sec"] = int32_t( 123 );
  msg["t"]["nanosec"] = uint32_t( 456 );
  msg["d"]["sec"] = int32_t( 789 );
  msg["d"]["nanosec"] = uint32_t( 101 );

  YAML::Node node = message_to_yaml( msg );
  EXPECT_EQ( node["t"]["sec"].as<int32_t>(), 123 );
  EXPECT_EQ( node["t"]["nanosec"].as<uint32_t>(), 456u );
  EXPECT_EQ( node["d"]["sec"].as<int32_t>(), 789 );
  EXPECT_EQ( node["d"]["nanosec"].as<uint32_t>(), 101u );

  CompoundMessage msg2 = fish.create_message( "ros_babel_fish_test_msgs/msg/TestMessage" );
  yaml_to_message( node, msg2 );
  EXPECT_EQ( msg2["t"]["sec"].value<int32_t>(), 123 );
  EXPECT_EQ( msg2["t"]["nanosec"].value<uint32_t>(), 456u );
  EXPECT_EQ( msg2["d"]["sec"].value<int32_t>(), 789 );
  EXPECT_EQ( msg2["d"]["nanosec"].value<uint32_t>(), 101u );
}

TEST_F( YamlSerializationTest, negativeDuration )
{
  CompoundMessage msg = fish.create_message( "ros_babel_fish_test_msgs/msg/TestMessage" );
  // -0.5 s: sec = -1, nanosec = 500000000 (canonical ROS representation)
  msg["d"]["sec"] = int32_t( -1 );
  msg["d"]["nanosec"] = uint32_t( 500000000 );

  YAML::Node node = message_to_yaml( msg );
  EXPECT_EQ( node["d"]["sec"].as<int32_t>(), -1 );
  EXPECT_EQ( node["d"]["nanosec"].as<uint32_t>(), 500000000u );

  CompoundMessage msg2 = fish.create_message( "ros_babel_fish_test_msgs/msg/TestMessage" );
  yaml_to_message( node, msg2 );
  EXPECT_EQ( msg2["d"]["sec"].value<int32_t>(), -1 );
  EXPECT_EQ( msg2["d"]["nanosec"].value<uint32_t>(), 500000000u );
}

TEST_F( YamlSerializationTest, nestedMessage )
{
  CompoundMessage msg = fish.create_message( "geometry_msgs/msg/Pose" );
  msg["position"]["x"] = 1.0;
  msg["position"]["y"] = 2.0;
  msg["position"]["z"] = 3.0;
  msg["orientation"]["w"] = 1.0;

  YAML::Node node = message_to_yaml( msg );
  EXPECT_TRUE( node["position"].IsMap() );
  EXPECT_TRUE( node["orientation"].IsMap() );

  CompoundMessage msg2 = fish.create_message( "geometry_msgs/msg/Pose" );
  yaml_to_message( node, msg2 );
  EXPECT_DOUBLE_EQ( msg2["position"]["x"].value<double>(), 1.0 );
  EXPECT_DOUBLE_EQ( msg2["position"]["y"].value<double>(), 2.0 );
  EXPECT_DOUBLE_EQ( msg2["position"]["z"].value<double>(), 3.0 );
  EXPECT_DOUBLE_EQ( msg2["orientation"]["w"].value<double>(), 1.0 );
}

TEST_F( YamlSerializationTest, compoundArray )
{
  CompoundMessage msg = fish.create_message( "ros_babel_fish_test_msgs/msg/TestMessage" );
  auto &arr = msg["point_arr"].as<ArrayMessageBase>().as<CompoundArrayMessage>();
  arr.resize( 2 );
  arr[0]["x"] = 1.0;
  arr[0]["y"] = 2.0;
  arr[0]["z"] = 3.0;
  arr[1]["x"] = 4.0;
  arr[1]["y"] = 5.0;
  arr[1]["z"] = 6.0;

  YAML::Node node = message_to_yaml( msg );
  ASSERT_TRUE( node["point_arr"].IsSequence() );
  ASSERT_EQ( node["point_arr"].size(), 2u );

  CompoundMessage msg2 = fish.create_message( "ros_babel_fish_test_msgs/msg/TestMessage" );
  yaml_to_message( node, msg2 );
  auto &arr2 = msg2["point_arr"].as<ArrayMessageBase>().as<CompoundArrayMessage>();
  ASSERT_EQ( arr2.size(), 2u );
  EXPECT_DOUBLE_EQ( arr2[0]["x"].value<double>(), 1.0 );
  EXPECT_DOUBLE_EQ( arr2[1]["z"].value<double>(), 6.0 );
}

TEST_F( YamlSerializationTest, boundedArrayThrows )
{
  // float64[<=16] in TestArray — build a sequence with 17 elements
  YAML::Node node;
  node["float64s"] = YAML::Node( YAML::NodeType::Sequence );
  for ( int i = 0; i < 17; ++i ) node["float64s"].push_back( static_cast<double>( i ) );

  CompoundMessage msg = fish.create_message( "ros_babel_fish_test_msgs/msg/TestArray" );
  EXPECT_THROW( ( yaml_to_message<BoundsCheckBehavior::Throw>( node, msg ) ),
                ros_babel_fish::BabelFishException );
}

TEST_F( YamlSerializationTest, boundedArrayClamp )
{
  // float64[<=16] — 20 elements; Clamp should cap at 16
  YAML::Node node;
  node["float64s"] = YAML::Node( YAML::NodeType::Sequence );
  for ( int i = 0; i < 20; ++i ) node["float64s"].push_back( static_cast<double>( i ) );

  CompoundMessage msg = fish.create_message( "ros_babel_fish_test_msgs/msg/TestArray" );
  ASSERT_NO_THROW( ( yaml_to_message<BoundsCheckBehavior::Clamp>( node, msg ) ) );

  const auto &arr = msg["float64s"].as<ArrayMessageBase>().as<BoundedArrayMessage<double>>();
  EXPECT_EQ( arr.size(), 16u );
  EXPECT_DOUBLE_EQ( arr[0], 0.0 );
  EXPECT_DOUBLE_EQ( arr[15], 15.0 );
}

TEST_F( YamlSerializationTest, int8RangeCheck )
{
  // int8 field: value 200 is out of range for int8, should throw
  YAML::Node node;
  node["int8s"] = YAML::Node( YAML::NodeType::Sequence );
  node["int8s"].push_back( 200 );

  CompoundMessage msg = fish.create_message( "ros_babel_fish_test_msgs/msg/TestArray" );
  EXPECT_THROW( yaml_to_message( node, msg ), ros_babel_fish::BabelFishException );
}

TEST_F( YamlSerializationTest, invalidYamlTimeFastPathThrowsBabelFishException )
{
  YAML::Node node;
  node["t"]["sec"] = "invalid";

  CompoundMessage msg = fish.create_message( "ros_babel_fish_test_msgs/msg/TestMessage" );
  try {
    yaml_to_message( node, msg );
    FAIL() << "Should have thrown BabelFishException";
  } catch ( const BabelFishException & ) {
  } catch ( const YAML::Exception &e ) {
    FAIL() << "Expected wrapped BabelFishException but caught raw YAML exception: " << e.what();
  } catch ( const std::exception &e ) {
    FAIL() << "Expected BabelFishException but caught: " << e.what();
  }
}

TEST_F( YamlSerializationTest, invalidYamlDurationFastPathThrowsBabelFishException )
{
  YAML::Node node;
  node["d"]["nanosec"] = "invalid";

  CompoundMessage msg = fish.create_message( "ros_babel_fish_test_msgs/msg/TestMessage" );
  try {
    yaml_to_message( node, msg );
    FAIL() << "Should have thrown BabelFishException";
  } catch ( const BabelFishException & ) {
  } catch ( const YAML::Exception &e ) {
    FAIL() << "Expected wrapped BabelFishException but caught raw YAML exception: " << e.what();
  } catch ( const std::exception &e ) {
    FAIL() << "Expected BabelFishException but caught: " << e.what();
  }
}

TEST_F( YamlSerializationTest, nullYamlTimeAndDurationFieldsDefaultToZero )
{
  YAML::Node node;
  node["t"]["sec"] = YAML::Node();
  node["t"]["nanosec"] = YAML::Node();
  node["d"]["sec"] = YAML::Node();
  node["d"]["nanosec"] = YAML::Node();

  CompoundMessage msg = fish.create_message( "ros_babel_fish_test_msgs/msg/TestMessage" );
  msg["t"]["sec"] = int32_t( 99 );
  msg["t"]["nanosec"] = uint32_t( 88 );
  msg["d"]["sec"] = int32_t( 77 );
  msg["d"]["nanosec"] = uint32_t( 66 );

  ASSERT_NO_THROW( yaml_to_message( node, msg ) );
  EXPECT_EQ( msg["t"]["sec"].value<int32_t>(), 0 );
  EXPECT_EQ( msg["t"]["nanosec"].value<uint32_t>(), 0u );
  EXPECT_EQ( msg["d"]["sec"].value<int32_t>(), 0 );
  EXPECT_EQ( msg["d"]["nanosec"].value<uint32_t>(), 0u );
}

TEST_F( YamlSerializationTest, nullValuesIgnored )
{
  CompoundMessage msg = fish.create_message( "ros_babel_fish_test_msgs/msg/TestMessage" );
  msg["str"] = "initial";
  msg["i32"] = (int32_t)42;

  YAML::Node node;
  node["str"] = YAML::Node(); // Explicit null
  node["i32"] = YAML::Node();
  node["f64"] = 1.5;

  yaml_to_message( node, msg );
  EXPECT_EQ( msg["str"].value<std::string>(), "initial" );
  EXPECT_EQ( msg["i32"].value<int32_t>(), 42 );
  EXPECT_NEAR( msg["f64"].value<double>(), 1.5, 1e-6 );

  // Array with nulls
  CompoundMessage msg_arr = fish.create_message( "ros_babel_fish_test_msgs/msg/TestArray" );
  auto &uint32_array = msg_arr["uint32s"].as<ArrayMessage<uint32_t>>();
  uint32_array.resize( 2 );
  uint32_array.assign( 0, 100 );
  uint32_array.assign( 1, 200 );

  YAML::Node node_arr;
  node_arr["uint32s"].push_back( YAML::Node() ); // Explicit null
  node_arr["uint32s"].push_back( 300 );

  yaml_to_message( node_arr, msg_arr );
  EXPECT_EQ( uint32_array[0], 100 );
  EXPECT_EQ( uint32_array[1], 300 );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
