// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish_tools/nlohmann_json_serialization.hpp"

#include <ros_babel_fish/babel_fish.hpp>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

using namespace ros_babel_fish;
using namespace ros_babel_fish_tools;

class JsonSerializationTest : public ::testing::Test
{
protected:
  BabelFish fish;
};

TEST_F( JsonSerializationTest, primitiveValues )
{
  auto msg = fish.create_message_shared( "geometry_msgs/msg/Vector3" );
  ( *msg )["x"] = 1.5;
  ( *msg )["y"] = -2.3;
  ( *msg )["z"] = 42.0;

  json j = compound_message_to_json( *msg );
  EXPECT_TRUE( j.is_object() );
  EXPECT_DOUBLE_EQ( j["x"].get<double>(), 1.5 );
  EXPECT_DOUBLE_EQ( j["y"].get<double>(), -2.3 );
  EXPECT_DOUBLE_EQ( j["z"].get<double>(), 42.0 );
}

TEST_F( JsonSerializationTest, nestedMessage )
{
  auto msg = fish.create_message_shared( "geometry_msgs/msg/Pose" );
  ( *msg )["position"]["x"] = 1.0;
  ( *msg )["position"]["y"] = 2.0;
  ( *msg )["position"]["z"] = 3.0;
  ( *msg )["orientation"]["x"] = 0.0;
  ( *msg )["orientation"]["y"] = 0.0;
  ( *msg )["orientation"]["z"] = 0.0;
  ( *msg )["orientation"]["w"] = 1.0;

  json j = compound_message_to_json( *msg );
  EXPECT_TRUE( j.is_object() );
  EXPECT_TRUE( j["position"].is_object() );
  EXPECT_TRUE( j["orientation"].is_object() );
  EXPECT_DOUBLE_EQ( j["position"]["x"].get<double>(), 1.0 );
  EXPECT_DOUBLE_EQ( j["position"]["y"].get<double>(), 2.0 );
  EXPECT_DOUBLE_EQ( j["position"]["z"].get<double>(), 3.0 );
  EXPECT_DOUBLE_EQ( j["orientation"]["w"].get<double>(), 1.0 );
}

TEST_F( JsonSerializationTest, timeAndDuration )
{
  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestMessage" );
  ( *msg )["t"]["sec"] = 123;
  ( *msg )["t"]["nanosec"] = 456u;
  ( *msg )["d"]["sec"] = 789;
  ( *msg )["d"]["nanosec"] = 101u;

  json j = compound_message_to_json( *msg );
  EXPECT_EQ( j["t"]["sec"].get<int32_t>(), 123 );
  EXPECT_EQ( j["t"]["nanosec"].get<uint32_t>(), 456u );
  EXPECT_EQ( j["d"]["sec"].get<int32_t>(), 789 );
  EXPECT_EQ( j["d"]["nanosec"].get<uint32_t>(), 101u );
}

TEST_F( JsonSerializationTest, stringValues )
{
  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestMessage" );
  ( *msg )["str"] = "hello world";

  json j = compound_message_to_json( *msg );
  EXPECT_EQ( j["str"].get<std::string>(), "hello world" );
}

TEST_F( JsonSerializationTest, boolValue )
{
  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestMessage" );
  ( *msg )["b"] = true;

  json j = compound_message_to_json( *msg );
  EXPECT_TRUE( j["b"].is_boolean() );
  EXPECT_TRUE( j["b"].get<bool>() );
}

TEST_F( JsonSerializationTest, integerTypes )
{
  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestMessage" );
  ( *msg )["ui8"] = uint8_t( 255 );
  ( *msg )["ui16"] = uint16_t( 65535 );
  ( *msg )["ui32"] = uint32_t( 100000 );
  ( *msg )["ui64"] = uint64_t( 123456789012345ULL );
  ( *msg )["i8"] = int8_t( -128 );
  ( *msg )["i16"] = int16_t( -1000 );
  ( *msg )["i32"] = int32_t( -42 );
  ( *msg )["i64"] = int64_t( -9876543210LL );

  json j = compound_message_to_json( *msg );
  EXPECT_EQ( j["ui8"].get<uint8_t>(), 255 );
  EXPECT_EQ( j["ui16"].get<uint16_t>(), 65535 );
  EXPECT_EQ( j["ui32"].get<uint32_t>(), 100000u );
  EXPECT_EQ( j["ui64"].get<uint64_t>(), 123456789012345ULL );
  EXPECT_EQ( j["i8"].get<int8_t>(), -128 );
  EXPECT_EQ( j["i16"].get<int16_t>(), -1000 );
  EXPECT_EQ( j["i32"].get<int32_t>(), -42 );
  EXPECT_EQ( j["i64"].get<int64_t>(), -9876543210LL );
}

TEST_F( JsonSerializationTest, floatTypes )
{
  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestMessage" );
  ( *msg )["f32"] = 3.14f;
  ( *msg )["f64"] = 2.71828;

  json j = compound_message_to_json( *msg );
  EXPECT_FLOAT_EQ( j["f32"].get<float>(), 3.14f );
  EXPECT_DOUBLE_EQ( j["f64"].get<double>(), 2.71828 );
}

TEST_F( JsonSerializationTest, primitiveArrays )
{
  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestArray" );
  auto &bools = ( *msg )["bools"].as<ArrayMessageBase>().as<ArrayMessage<bool>>();
  bools.resize( 3 );
  bools.assign( 0, true );
  bools.assign( 1, false );
  bools.assign( 2, true );

  auto &uint32s = ( *msg )["uint32s"].as<ArrayMessageBase>().as<ArrayMessage<uint32_t>>();
  uint32s.resize( 2 );
  uint32s.assign( 0, 100u );
  uint32s.assign( 1, 200u );

  auto &int32s = ( *msg )["int32s"].as<ArrayMessageBase>().as<ArrayMessage<int32_t>>();
  int32s.resize( 3 );
  int32s.assign( 0, 10 );
  int32s.assign( 1, 20 );
  int32s.assign( 2, 30 );

  json j = compound_message_to_json( *msg );
  ASSERT_TRUE( j["bools"].is_array() );
  ASSERT_EQ( j["bools"].size(), 3u );
  EXPECT_TRUE( j["bools"][0].get<bool>() );
  EXPECT_FALSE( j["bools"][1].get<bool>() );
  EXPECT_TRUE( j["bools"][2].get<bool>() );

  ASSERT_TRUE( j["uint32s"].is_array() );
  ASSERT_EQ( j["uint32s"].size(), 2u );
  EXPECT_EQ( j["uint32s"][0].get<uint32_t>(), 100u );
  EXPECT_EQ( j["uint32s"][1].get<uint32_t>(), 200u );

  ASSERT_TRUE( j["int32s"].is_array() );
  ASSERT_EQ( j["int32s"].size(), 3u );
  EXPECT_EQ( j["int32s"][0].get<int32_t>(), 10 );
  EXPECT_EQ( j["int32s"][1].get<int32_t>(), 20 );
  EXPECT_EQ( j["int32s"][2].get<int32_t>(), 30 );
}

TEST_F( JsonSerializationTest, stringArrays )
{
  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestArray" );
  auto &strings = ( *msg )["strings"].as<ArrayMessageBase>();
  auto &str_arr = strings.as<ArrayMessage<std::string>>();
  str_arr.resize( 2 );
  str_arr.assign( 0, "hello" );
  str_arr.assign( 1, "world" );

  json j = compound_message_to_json( *msg );
  ASSERT_TRUE( j["strings"].is_array() );
  ASSERT_EQ( j["strings"].size(), 2u );
  EXPECT_EQ( j["strings"][0].get<std::string>(), "hello" );
  EXPECT_EQ( j["strings"][1].get<std::string>(), "world" );
}

TEST_F( JsonSerializationTest, compoundArrays )
{
  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestMessage" );
  auto &arr = ( *msg )["point_arr"].as<ArrayMessageBase>();
  auto &compound_arr = arr.as<CompoundArrayMessage>();
  compound_arr.resize( 2 );
  compound_arr[0]["x"] = 1.0;
  compound_arr[0]["y"] = 2.0;
  compound_arr[0]["z"] = 3.0;
  compound_arr[1]["x"] = 4.0;
  compound_arr[1]["y"] = 5.0;
  compound_arr[1]["z"] = 6.0;

  json j = compound_message_to_json( *msg );
  ASSERT_TRUE( j["point_arr"].is_array() );
  ASSERT_EQ( j["point_arr"].size(), 2u );
  EXPECT_DOUBLE_EQ( j["point_arr"][0]["x"].get<double>(), 1.0 );
  EXPECT_DOUBLE_EQ( j["point_arr"][1]["z"].get<double>(), 6.0 );
}

TEST_F( JsonSerializationTest, roundTripVector3 )
{
  auto msg = fish.create_message_shared( "geometry_msgs/msg/Vector3" );
  ( *msg )["x"] = 1.5;
  ( *msg )["y"] = -2.3;
  ( *msg )["z"] = 42.0;

  json j = compound_message_to_json( *msg );

  auto msg2 = fish.create_message_shared( "geometry_msgs/msg/Vector3" );
  json_to_message( j, *msg2 );

  EXPECT_DOUBLE_EQ( ( *msg2 )["x"].value<double>(), 1.5 );
  EXPECT_DOUBLE_EQ( ( *msg2 )["y"].value<double>(), -2.3 );
  EXPECT_DOUBLE_EQ( ( *msg2 )["z"].value<double>(), 42.0 );
}

TEST_F( JsonSerializationTest, roundTripPose )
{
  auto msg = fish.create_message_shared( "geometry_msgs/msg/Pose" );
  ( *msg )["position"]["x"] = 1.0;
  ( *msg )["position"]["y"] = 2.0;
  ( *msg )["position"]["z"] = 3.0;
  ( *msg )["orientation"]["w"] = 1.0;

  json j = compound_message_to_json( *msg );
  auto msg2 = fish.create_message_shared( "geometry_msgs/msg/Pose" );
  json_to_message( j, *msg2 );

  EXPECT_DOUBLE_EQ( ( *msg2 )["position"]["x"].value<double>(), 1.0 );
  EXPECT_DOUBLE_EQ( ( *msg2 )["position"]["y"].value<double>(), 2.0 );
  EXPECT_DOUBLE_EQ( ( *msg2 )["position"]["z"].value<double>(), 3.0 );
  EXPECT_DOUBLE_EQ( ( *msg2 )["orientation"]["w"].value<double>(), 1.0 );
}

TEST_F( JsonSerializationTest, roundTripTestMessage )
{
  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestMessage" );
  ( *msg )["b"] = true;
  ( *msg )["ui8"] = uint8_t( 42 );
  ( *msg )["i32"] = int32_t( -12345 );
  ( *msg )["f64"] = 3.14159;
  ( *msg )["str"] = "test string";
  ( *msg )["t"]["sec"] = 100;
  ( *msg )["t"]["nanosec"] = 500u;

  json j = compound_message_to_json( *msg );
  auto msg2 = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestMessage" );
  json_to_message( j, *msg2 );

  EXPECT_TRUE( ( *msg2 )["b"].value<bool>() );
  EXPECT_EQ( ( *msg2 )["ui8"].value<uint8_t>(), 42 );
  EXPECT_EQ( ( *msg2 )["i32"].value<int32_t>(), -12345 );
  EXPECT_DOUBLE_EQ( ( *msg2 )["f64"].value<double>(), 3.14159 );
  EXPECT_EQ( ( *msg2 )["str"].value<std::string>(), "test string" );
  EXPECT_EQ( ( *msg2 )["t"]["sec"].value<int32_t>(), 100 );
  EXPECT_EQ( ( *msg2 )["t"]["nanosec"].value<uint32_t>(), 500u );
}

TEST_F( JsonSerializationTest, roundTripArrays )
{
  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestArray" );
  auto &int32s = ( *msg )["int32s"].as<ArrayMessageBase>().as<ArrayMessage<int32_t>>();
  int32s.resize( 3 );
  int32s.assign( 0, 100 );
  int32s.assign( 1, 200 );
  int32s.assign( 2, 300 );

  auto &strings = ( *msg )["strings"].as<ArrayMessageBase>().as<ArrayMessage<std::string>>();
  strings.resize( 2 );
  strings.assign( 0, "foo" );
  strings.assign( 1, "bar" );

  json j = compound_message_to_json( *msg );
  auto msg2 = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestArray" );
  json_to_message( j, *msg2 );

  auto &int32s2 = ( *msg2 )["int32s"].as<ArrayMessageBase>().as<ArrayMessage<int32_t>>();
  ASSERT_EQ( int32s2.size(), 3u );
  EXPECT_EQ( int32s2[0], 100 );
  EXPECT_EQ( int32s2[1], 200 );
  EXPECT_EQ( int32s2[2], 300 );

  auto &strings2 = ( *msg2 )["strings"].as<ArrayMessageBase>().as<ArrayMessage<std::string>>();
  ASSERT_EQ( strings2.size(), 2u );
  EXPECT_EQ( strings2[0], "foo" );
  EXPECT_EQ( strings2[1], "bar" );
}

TEST_F( JsonSerializationTest, roundTripCompoundArray )
{
  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestMessage" );
  auto &arr = ( *msg )["point_arr"].as<ArrayMessageBase>().as<CompoundArrayMessage>();
  arr.resize( 2 );
  arr[0]["x"] = 10.0;
  arr[0]["y"] = 20.0;
  arr[0]["z"] = 30.0;
  arr[1]["x"] = 40.0;
  arr[1]["y"] = 50.0;
  arr[1]["z"] = 60.0;

  json j = compound_message_to_json( *msg );
  auto msg2 = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestMessage" );
  json_to_message( j, *msg2 );

  auto &arr2 = ( *msg2 )["point_arr"].as<ArrayMessageBase>().as<CompoundArrayMessage>();
  ASSERT_EQ( arr2.size(), 2u );
  EXPECT_DOUBLE_EQ( arr2[0]["x"].value<double>(), 10.0 );
  EXPECT_DOUBLE_EQ( arr2[1]["z"].value<double>(), 60.0 );
}

TEST_F( JsonSerializationTest, jsonToMessageFromType )
{
  json j = { { "x", 5.5 }, { "y", -3.3 }, { "z", 0.0 } };
  auto msg = fish.create_message_shared( "geometry_msgs/msg/Vector3" );
  json_to_message( j, *msg );
  EXPECT_DOUBLE_EQ( ( *msg )["x"].value<double>(), 5.5 );
  EXPECT_DOUBLE_EQ( ( *msg )["y"].value<double>(), -3.3 );
  EXPECT_DOUBLE_EQ( ( *msg )["z"].value<double>(), 0.0 );
}

TEST_F( JsonSerializationTest, partialJsonIgnoresMissingFields )
{
  // Only set x, leave y and z at defaults
  json j = { { "x", 99.0 } };
  auto msg = fish.create_message_shared( "geometry_msgs/msg/Vector3" );
  json_to_message( j, *msg );
  EXPECT_DOUBLE_EQ( ( *msg )["x"].value<double>(), 99.0 );
  EXPECT_DOUBLE_EQ( ( *msg )["y"].value<double>(), 0.0 );
  EXPECT_DOUBLE_EQ( ( *msg )["z"].value<double>(), 0.0 );
}

TEST_F( JsonSerializationTest, extraJsonFieldsIgnored )
{
  json j = { { "x", 1.0 }, { "y", 2.0 }, { "z", 3.0 }, { "extra_field", "ignored" } };
  auto msg = fish.create_message_shared( "geometry_msgs/msg/Vector3" );
  json_to_message( j, *msg );
  EXPECT_DOUBLE_EQ( ( *msg )["x"].value<double>(), 1.0 );
}

TEST_F( JsonSerializationTest, invalidJsonTypeThrows )
{
  json j = json::array( { 1, 2, 3 } );
  auto msg = fish.create_message_shared( "geometry_msgs/msg/Vector3" );
  EXPECT_THROW( json_to_message( j, *msg ), BabelFishException );

  auto other_msg = fish.create_message_shared( "geometry_msgs/msg/Pose" );
  j = message_to_json( *other_msg );
  j["position"]["x"] = "invalid type";
  try {
    json_to_message( j, *other_msg );
    FAIL() << "Should have thrown BabelFishException";
  } catch ( const BabelFishException &e ) {
    EXPECT_TRUE( std::string( e.what() ).find( "position.x" ) != std::string::npos )
        << "Exception message should contain path 'position.x', but was: " << e.what();
  } catch ( const std::exception &e ) {
    FAIL() << "Expected BabelFishException but caught: " << e.what();
  } catch ( ... ) {
    FAIL() << "Expected BabelFishException but caught an unknown exception type";
  }
  j["position"] = "invalid type";
  try {
    json_to_message( j, *other_msg );
    FAIL() << "Should have thrown BabelFishException";
  } catch ( const BabelFishException &e ) {
    EXPECT_TRUE( std::string( e.what() ).find( "position" ) != std::string::npos )
        << "Exception message should contain path 'position', but was: " << e.what();
  } catch ( const std::exception &e ) {
    FAIL() << "Expected BabelFishException but caught: " << e.what();
  } catch ( ... ) {
    FAIL() << "Expected BabelFishException but caught an unknown exception type";
  }
}

TEST_F( JsonSerializationTest, messageToJsonViaBaseClass )
{
  auto msg = fish.create_message_shared( "geometry_msgs/msg/Vector3" );
  ( *msg )["x"] = 7.7;
  const Message &base = *msg;
  json j = message_to_json( base );
  EXPECT_DOUBLE_EQ( j["x"].get<double>(), 7.7 );
}

TEST_F( JsonSerializationTest, negativeDuration )
{
  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestMessage" );
  // -0.5 s: sec = -1, nanosec = 500000000 (canonical ROS representation)
  ( *msg )["d"]["sec"] = int32_t( -1 );
  ( *msg )["d"]["nanosec"] = uint32_t( 500000000 );

  json j = compound_message_to_json( *msg );
  EXPECT_EQ( j["d"]["sec"].get<int32_t>(), -1 );
  EXPECT_EQ( j["d"]["nanosec"].get<uint32_t>(), 500000000u );

  auto msg2 = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestMessage" );
  json_to_message( j, *msg2 );
  EXPECT_EQ( ( *msg2 )["d"]["sec"].value<int32_t>(), -1 );
  EXPECT_EQ( ( *msg2 )["d"]["nanosec"].value<uint32_t>(), 500000000u );
}

TEST_F( JsonSerializationTest, supplementaryUnicode )
{
  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestMessage" );
  // U+1F600 GRINNING FACE — requires 4-byte UTF-8
  const std::string emoji_utf8 = "\xF0\x9F\x98\x80";
  ( *msg )["str"] = emoji_utf8;

  json j = compound_message_to_json( *msg );
  EXPECT_EQ( j["str"].get<std::string>(), emoji_utf8 );

  auto msg2 = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestMessage" );
  json_to_message( j, *msg2 );
  EXPECT_EQ( ( *msg2 )["str"].value<std::string>(), emoji_utf8 );
}

TEST_F( JsonSerializationTest, boundedArrayThrows )
{
  // float64[<=16] in TestArray — try to insert 17 elements
  json j = json::object();
  j["float64s"] = json::array();
  for ( int i = 0; i < 17; ++i ) j["float64s"].push_back( static_cast<double>( i ) );

  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestArray" );
  EXPECT_THROW( ( json_to_message<BoundsCheckBehavior::Throw>( j, *msg ) ),
                ros_babel_fish::BabelFishException );
}

TEST_F( JsonSerializationTest, boundedArrayClamp )
{
  // float64[<=16] — insert 20 elements; Clamp should cap at 16
  json j = json::object();
  j["float64s"] = json::array();
  for ( int i = 0; i < 20; ++i ) j["float64s"].push_back( static_cast<double>( i ) );

  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestArray" );
  ASSERT_NO_THROW( ( json_to_message<BoundsCheckBehavior::Clamp>( j, *msg ) ) );

  const auto &arr = ( *msg )["float64s"]
                        .as<ros_babel_fish::ArrayMessageBase>()
                        .as<ros_babel_fish::BoundedArrayMessage<double>>();
  EXPECT_EQ( arr.size(), 16u );
  EXPECT_DOUBLE_EQ( arr[0], 0.0 );
  EXPECT_DOUBLE_EQ( arr[15], 15.0 );
}

TEST_F( JsonSerializationTest, shortJsonLeavesRemainingFixedPrimitiveArrayElementsUnchanged )
{
  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestArray" );
  json j = { { "uint16s", { 1, 2 } } };

  ASSERT_NO_THROW( json_to_message( j, *msg ) );
  json round_trip = compound_message_to_json( *msg );
  ASSERT_TRUE( round_trip["uint16s"].is_array() );
  EXPECT_EQ( round_trip["uint16s"].size(), 32u );
  EXPECT_EQ( round_trip["uint16s"][0].get<uint16_t>(), 1 );
  EXPECT_EQ( round_trip["uint16s"][1].get<uint16_t>(), 2 );
  EXPECT_EQ( round_trip["uint16s"][2].get<uint16_t>(), 0 );
}

TEST_F( JsonSerializationTest, shortJsonLeavesRemainingFixedCompoundArrayElementsUnchanged )
{
  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestArray" );
  json j = { { "durations", { { { "sec", 1 }, { "nanosec", 2 } } } } };

  ASSERT_NO_THROW( json_to_message( j, *msg ) );
  json round_trip = compound_message_to_json( *msg );
  ASSERT_TRUE( round_trip["durations"].is_array() );
  EXPECT_EQ( round_trip["durations"].size(), 12u );
  EXPECT_EQ( round_trip["durations"][0]["sec"].get<int32_t>(), 1 );
  EXPECT_EQ( round_trip["durations"][0]["nanosec"].get<uint32_t>(), 2u );
  EXPECT_EQ( round_trip["durations"][1]["sec"].get<int32_t>(), 0 );
}

TEST_F( JsonSerializationTest, invalidJsonTimeFastPathThrowsBabelFishException )
{
  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestMessage" );
  json j = { { "t", { { "sec", "invalid" } } } };

  try {
    json_to_message( j, *msg );
    FAIL() << "Should have thrown BabelFishException";
  } catch ( const BabelFishException & ) {
  } catch ( const nlohmann::json::exception &e ) {
    FAIL() << "Expected wrapped BabelFishException but caught raw json exception: " << e.what();
  } catch ( const std::exception &e ) {
    FAIL() << "Expected BabelFishException but caught: " << e.what();
  }
}

TEST_F( JsonSerializationTest, invalidJsonDurationFastPathThrowsBabelFishException )
{
  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestMessage" );
  json j = { { "d", { { "nanosec", "invalid" } } } };

  try {
    json_to_message( j, *msg );
    FAIL() << "Should have thrown BabelFishException";
  } catch ( const BabelFishException & ) {
  } catch ( const nlohmann::json::exception &e ) {
    FAIL() << "Expected wrapped BabelFishException but caught raw json exception: " << e.what();
  } catch ( const std::exception &e ) {
    FAIL() << "Expected BabelFishException but caught: " << e.what();
  }
}

TEST_F( JsonSerializationTest, nullJsonTimeAndDurationFieldsDefaultToZero )
{
  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestMessage" );
  ( *msg )["t"]["sec"] = int32_t( 99 );
  ( *msg )["t"]["nanosec"] = uint32_t( 88 );
  ( *msg )["d"]["sec"] = int32_t( 77 );
  ( *msg )["d"]["nanosec"] = uint32_t( 66 );

  json j = {
      { "t", { { "sec", nullptr }, { "nanosec", nullptr } } },
      { "d", { { "sec", nullptr }, { "nanosec", nullptr } } },
  };

  ASSERT_NO_THROW( json_to_message( j, *msg ) );
  EXPECT_EQ( ( *msg )["t"]["sec"].value<int32_t>(), 0 );
  EXPECT_EQ( ( *msg )["t"]["nanosec"].value<uint32_t>(), 0u );
  EXPECT_EQ( ( *msg )["d"]["sec"].value<int32_t>(), 0 );
  EXPECT_EQ( ( *msg )["d"]["nanosec"].value<uint32_t>(), 0u );
}

TEST_F( JsonSerializationTest, nullValuesIgnored )
{
  auto msg = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestMessage" );
  ( *msg )["str"] = "initial";
  ( *msg )["i32"] = 42;

  json j = { { "str", nullptr }, { "i32", nullptr }, { "f64", 1.5 } };
  json_to_message( j, *msg );

  EXPECT_EQ( ( *msg )["str"].value<std::string>(), "initial" );
  EXPECT_EQ( ( *msg )["i32"].value<int32_t>(), 42 );
  EXPECT_DOUBLE_EQ( ( *msg )["f64"].value<double>(), 1.5 );

  // Array with nulls
  auto msg_arr = fish.create_message_shared( "ros_babel_fish_test_msgs/msg/TestArray" );
  auto &uint32s = ( *msg_arr )["uint32s"].as<ArrayMessage<uint32_t>>();
  uint32s.resize( 2 );
  uint32s.assign( 0, 100 );
  uint32s.assign( 1, 200 );

  json j_arr = { { "uint32s", { nullptr, 300 } } };
  json_to_message( j_arr, *msg_arr );
  EXPECT_EQ( uint32s[0], 100 );
  EXPECT_EQ( uint32s[1], 300 );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
