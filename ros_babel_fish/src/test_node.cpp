//
// Created by stefan on 21.05.21.
//

#include "test_messages.h"
#include <rclcpp/rclcpp.hpp>
#include <ros_babel_fish/babel_fish.hpp>
#include <ros_babel_fish_test_msgs/msg/test_message.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals; // For 3s etc.
using namespace ros_babel_fish;

#define DECLARE_PUB( MSG_TYPE, NAME ) rclcpp::Publisher<MSG_TYPE>::SharedPtr NAME##_pub_
#define DEFINE_PUB( MSG_TYPE, NAME )                                                               \
  template<>                                                                                       \
  rclcpp::Publisher<MSG_TYPE>::SharedPtr TestNode::getPublisher<MSG_TYPE>()                        \
  {                                                                                                \
    return NAME##_pub_;                                                                            \
  }                                                                                                \
  template<>                                                                                       \
  std::string TestNode::getTopic<MSG_TYPE>()                                                       \
  {                                                                                                \
    return "~/" #NAME;                                                                             \
  }
#define INIT_PUB( MSG_TYPE, NAME ) NAME##_pub_ = this->create_publisher<MSG_TYPE>( "~/" #NAME, 1 )

class TestNode : public rclcpp::Node
{
public:
  TestNode() : Node( "ros_babel_fish_test" )
  {
    INIT_PUB( geometry_msgs::msg::PoseStamped, pose_stamped );
    INIT_PUB( ros_babel_fish_test_msgs::msg::TestMessage, test_message );
  }

  template<typename MessageType>
  bool testMessage()
  {
    rclcpp::GuardCondition::SharedPtr cond = std::make_shared<rclcpp::GuardCondition>();
    rclcpp::WaitSet set;
    set.add_guard_condition( cond );
    ros_babel_fish::CompoundMessage::SharedPtr message = nullptr;
    auto message_sub = fish.create_subscription(
        *this, getTopic<MessageType>(), 10,
        [&message, &cond]( ros_babel_fish::CompoundMessage::SharedPtr msg ) {
          message = msg;
          cond->trigger();
        } );
    MessageType test_message;
    initMessage( test_message );
    getPublisher<MessageType>()->publish( test_message );
    if ( set.wait( 5s ).kind() != rclcpp::WaitResultKind::Ready ) {
      std::cerr << "Timeout while waiting for message!" << std::endl;
      return false;
    }
    return messagesAreEqual( test_message, *message );
  }

private:
  template<typename MessageType>
  typename rclcpp::Publisher<MessageType>::SharedPtr getPublisher();

  template<typename MessageType>
  std::string getTopic();

  DECLARE_PUB( geometry_msgs::msg::PoseStamped, pose_stamped );
  DECLARE_PUB( ros_babel_fish_test_msgs::msg::TestMessage, test_message );

  ros_babel_fish::BabelFish fish;
};

// Workaround until GCC supports them in the class again (part of C++17 but should be applied retroactively)
DEFINE_PUB( geometry_msgs::msg::PoseStamped, pose_stamped )

DEFINE_PUB( ros_babel_fish_test_msgs::msg::TestMessage, test_message )

std::ostream &print( const std::string &txt, size_t width = 60, char fill_character = ' ' )
{
  std::cout << txt;
  if ( txt.length() >= width )
    return std::cout;
  std::cout << std::string( width - txt.length(), fill_character );
  return std::cout;
}

int printResult( bool failed )
{
  print( "", 80, '-' );
  std::cout << std::endl;
  if ( failed ) {
    std::cerr << "It seems like the middleware you are using is not supported by ROS2 Babel Fish!"
              << std::endl;
    std::cerr << "Feel free to make a PR or create an issue on GitHub if this middleware is "
                 "officially supported by ROS2 Babel Fish."
              << std::endl;
  } else {
    std::cout << "Everything is looking good! ROS2 Babel Fish should work for your setup!"
              << std::endl;
  }
  rclcpp::shutdown();
  return failed ? 1 : 0;
}

template<typename MessageType>
bool test( const std::shared_ptr<TestNode> &node, const std::string &type )
{
  std::cout << std::endl << ">>> Testing " << type << std::endl;
  std::shared_future<bool> message_result =
      std::async( std::launch::async, &TestNode::testMessage<MessageType>, node.get() );
  rclcpp::spin_until_future_complete( node, message_result );
  print( type + "...", 60, '.' );
  if ( !message_result.get() ) {
    std::cout << "FAIL" << std::endl;
    return false;
  } else {
    std::cout << ".OK " << std::endl;
  }
  return true;
}

int main( int argc, char *argv[] )
{
  rclcpp::init( argc, argv );
  std::cout << R"( ______  ______  ______    _______
/\  == \/\  __ \/\  ___\  /\____  \
\ \  __<\ \ \/\ \ \___  \ \/____/\ \
 \ \_\ \_\ \_____\/\_____\  /\  ____\
  \/_/ /_/\/_____/\/_____/  \ \ \___/_
                             \ \______\
                              \/______/
 ______  ______  ______  ______  __           ______ __  ______  __  __
/\  == \/\  __ \/\  == \/\  ___\/\ \         /\  ___/\ \/\  ___\/\ \_\ \
\ \  __<\ \  __ \ \  __<\ \  __\\ \ \____    \ \  __\ \ \ \___  \ \  __ \
 \ \_____\ \_\ \_\ \_____\ \_____\ \_____\    \ \_\  \ \_\/\_____\ \_\ \_\
  \/_____/\/_/\/_/\/_____/\/_____/\/_____/     \/_/   \/_/\/_____/\/_/\/_/
   by Stefan Fabian
--------------------------------------------------------------------------------
Test Node - Checks if ROS2 Babel Fish works with the middleware you are using.

)";
  print( "Creating node...", 60, '.' );
  auto node = std::make_shared<TestNode>();
  std::cout << ".OK" << std::endl;
  // Test message
  if ( !test<geometry_msgs::msg::PoseStamped>( node, "geometry_msgs/msg/PoseStamped" ) )
    return printResult( true );
  if ( !test<ros_babel_fish_test_msgs::msg::TestMessage>(
           node, "ros_babel_fish_test_msgs/msg/TestMessage" ) )
    return printResult( true );

  return printResult( false );
}
