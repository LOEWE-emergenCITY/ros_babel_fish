// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

// End-to-end tests for the echo tool: the executable (path passed in via ECHO_EXECUTABLE) is run
// against a publisher created by this test and its output is checked.

#include "test_helpers.hpp"

#include <ros_babel_fish_tools/third_party/nlohmann_json.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <yaml-cpp/yaml.h>

using namespace ros_babel_fish_tools_test;
using namespace std::chrono_literals;

namespace
{
std::shared_ptr<rclcpp::Node> node;
const std::string kTopicPrefix = "/rbf_tools_test/echo/";
const std::string kPoseType = "geometry_msgs/msg/PoseStamped";

geometry_msgs::msg::PoseStamped make_pose()
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp.sec = 1234;
  msg.header.stamp.nanosec = 5678;
  msg.pose.position.x = 1.5;
  msg.pose.position.y = -2.25;
  msg.pose.position.z = 3.0;
  msg.pose.orientation.w = 1.0;
  return msg;
}

//! Publishes @p msg on @p topic at 20 Hz until @p process exits (or @p timeout expires).
//! @return The exit code of the process or nullopt if it did not exit in time.
template<typename MessageT>
std::optional<int> publish_until_exit( const std::string &topic, const MessageT &msg,
                                       ToolProcess &process, std::chrono::milliseconds timeout = 15s )
{
  auto pub = node->create_publisher<MessageT>( topic, 10 );
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while ( std::chrono::steady_clock::now() < deadline ) {
    pub->publish( msg );
    if ( auto code = process.wait( 50ms ) )
      return code;
  }
  return std::nullopt;
}
} // namespace

TEST( EchoTool, printsFirstMessageAsCompactJson )
{
  const std::string topic = kTopicPrefix + "json";
  ToolProcess echo;
  ASSERT_TRUE( echo.start( ECHO_EXECUTABLE, { topic, kPoseType } ) );

  const auto code = publish_until_exit( topic, make_pose(), echo );
  ASSERT_TRUE( code.has_value() ) << "echo did not exit after receiving a message. stderr:\n"
                                  << echo.stderr_text();
  EXPECT_EQ( *code, 0 );

  const std::vector<std::string> lines = split_lines( echo.stdout_text() );
  ASSERT_EQ( lines.size(), 1u ) << "compact JSON must be a single line:\n" << echo.stdout_text();
  nlohmann::json j;
  ASSERT_NO_THROW( j = nlohmann::json::parse( lines[0] ) ) << lines[0];
  EXPECT_EQ( j["header"]["frame_id"], "map" );
  EXPECT_EQ( j["header"]["stamp"]["sec"], 1234 );
  EXPECT_EQ( j["header"]["stamp"]["nanosec"], 5678 );
  EXPECT_DOUBLE_EQ( j["pose"]["position"]["x"].get<double>(), 1.5 );
  EXPECT_DOUBLE_EQ( j["pose"]["position"]["y"].get<double>(), -2.25 );
  EXPECT_DOUBLE_EQ( j["pose"]["position"]["z"].get<double>(), 3.0 );
  EXPECT_DOUBLE_EQ( j["pose"]["orientation"]["w"].get<double>(), 1.0 );
}

TEST( EchoTool, prettyPrintsJson )
{
  const std::string topic = kTopicPrefix + "pretty";
  for ( const std::string flag : { "--pretty", "-p" } ) {
    ToolProcess echo;
    ASSERT_TRUE( echo.start( ECHO_EXECUTABLE, { topic, kPoseType, flag } ) );

    const auto code = publish_until_exit( topic, make_pose(), echo );
    ASSERT_TRUE( code.has_value() ) << echo.stderr_text();
    EXPECT_EQ( *code, 0 );

    const std::string out = echo.stdout_text();
    EXPECT_GT( split_lines( out ).size(), 1u ) << "pretty output spans multiple lines: " << out;
    EXPECT_NE( out.find( "    \"header\"" ), std::string::npos ) << "4-space indentation: " << out;
    nlohmann::json j;
    ASSERT_NO_THROW( j = nlohmann::json::parse( out ) ) << out;
    EXPECT_EQ( j["header"]["frame_id"], "map" );
    EXPECT_DOUBLE_EQ( j["pose"]["position"]["x"].get<double>(), 1.5 );
  }
}

TEST( EchoTool, printsYaml )
{
  const std::string topic = kTopicPrefix + "yaml";
  ToolProcess echo;
  ASSERT_TRUE( echo.start( ECHO_EXECUTABLE, { topic, kPoseType, "--yaml" } ) );

  const auto code = publish_until_exit( topic, make_pose(), echo );
  ASSERT_TRUE( code.has_value() ) << echo.stderr_text();
  EXPECT_EQ( *code, 0 );

  const std::string out = echo.stdout_text();
  YAML::Node n;
  ASSERT_NO_THROW( n = YAML::Load( out ) ) << out;
  ASSERT_TRUE( n.IsMap() ) << out;
  EXPECT_EQ( n["header"]["frame_id"].as<std::string>(), "map" );
  EXPECT_EQ( n["header"]["stamp"]["sec"].as<int>(), 1234 );
  EXPECT_DOUBLE_EQ( n["pose"]["position"]["y"].as<double>(), -2.25 );
  EXPECT_EQ( out.find( '{' ), std::string::npos ) << "block-style YAML, not JSON: " << out;
}

TEST( EchoTool, lastFormatFlagWins )
{
  const std::string topic = kTopicPrefix + "format_order";
  ToolProcess echo;
  ASSERT_TRUE( echo.start( ECHO_EXECUTABLE, { topic, kPoseType, "--yaml", "--json" } ) );

  const auto code = publish_until_exit( topic, make_pose(), echo );
  ASSERT_TRUE( code.has_value() ) << echo.stderr_text();
  EXPECT_EQ( *code, 0 );
  const std::vector<std::string> lines = split_lines( echo.stdout_text() );
  ASSERT_EQ( lines.size(), 1u ) << echo.stdout_text();
  nlohmann::json j;
  EXPECT_NO_THROW( j = nlohmann::json::parse( lines[0] ) ) << lines[0];
  EXPECT_TRUE( j.is_object() );
}

TEST( EchoTool, detectsTypeFromGraph )
{
  const std::string topic = kTopicPrefix + "autodetect";
  std_msgs::msg::Int32 msg;
  msg.data = -77;
  // The tool looks the type up in the graph and waits for a publisher to appear; advertising one
  // up front means the lookup does not have to wait.
  auto pub = node->create_publisher<std_msgs::msg::Int32>( topic, 10 );

  ToolProcess echo;
  ASSERT_TRUE( echo.start( ECHO_EXECUTABLE, { topic } ) );
  const auto code = publish_until_exit( topic, msg, echo );
  ASSERT_TRUE( code.has_value() ) << echo.stderr_text();
  EXPECT_EQ( *code, 0 );

  nlohmann::json j;
  ASSERT_NO_THROW( j = nlohmann::json::parse( echo.stdout_text() ) ) << echo.stdout_text();
  EXPECT_EQ( j["data"], -77 );
}

TEST( EchoTool, appliesRosArgs )
{
  // --ros-args must be stripped from the tool's own arguments and still take effect: remap the
  // subscribed topic and check the message arrives from the remapped one.
  const std::string topic = kTopicPrefix + "remap_from";
  const std::string target = kTopicPrefix + "remap_to";
  std_msgs::msg::Int32 msg;
  msg.data = 9;
  ToolProcess echo;
  ASSERT_TRUE( echo.start( ECHO_EXECUTABLE, { topic, "std_msgs/msg/Int32", "--ros-args", "-r",
                                              topic + ":=" + target } ) );
  const auto code = publish_until_exit( target, msg, echo );
  ASSERT_TRUE( code.has_value() ) << echo.stderr_text();
  EXPECT_EQ( *code, 0 );
  nlohmann::json j;
  ASSERT_NO_THROW( j = nlohmann::json::parse( echo.stdout_text() ) ) << echo.stdout_text();
  EXPECT_EQ( j["data"], 9 );
}

TEST( EchoTool, helpExitsSuccessfully ) { expect_help_on_stderr( ECHO_EXECUTABLE, "--yaml" ); }

TEST( EchoTool, rejectsInvalidArguments )
{
  expect_rejects_arguments(
      ECHO_EXECUTABLE,
      {
          { {}, "Usage:" },
          { { kTopicPrefix + "x", kPoseType, "--bogus" }, "Unknown option: --bogus" },
          { { kTopicPrefix + "x", kPoseType, "extra" }, "Too many positional arguments: extra" },
          { { kTopicPrefix + "x", "no_such_pkg/msg/Nope" }, "Failed to create subscription" },
      } );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  node = std::make_shared<rclcpp::Node>( "echo_tool_test" );
  const int result = RUN_ALL_TESTS();
  node.reset();
  rclcpp::shutdown();
  return result;
}
