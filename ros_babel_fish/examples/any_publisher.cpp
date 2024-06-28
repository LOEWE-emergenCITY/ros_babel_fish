// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <ros_babel_fish/babel_fish.hpp>
#include <ros_babel_fish/messages/array_message.hpp>
#include <ros_babel_fish/messages/compound_message.hpp>
#include <ros_babel_fish/messages/value_message.hpp>

#include <rclcpp/rclcpp.hpp>

/*!
 * The following example demonstrates how messages can be published without knowing their type at compile time.
 */

using namespace std::chrono_literals; // So we can use s to specify the timer interval in seconds

class AnyPublisher : public rclcpp::Node
{
public:
  AnyPublisher() : Node( "any_publisher" ), count_( 0 ) { }

  void init()
  {
    fish_ = ros_babel_fish::BabelFish::make_shared();
    pub_string_ = fish_->create_publisher( *this, "/string", "std_msgs/msg/String", 10,
                                           rclcpp::PublisherOptions() );
    pub_pose_ = fish_->create_publisher( *this, "/pose", "geometry_msgs/msg/Pose", 10,
                                         rclcpp::PublisherOptions() );
    pub_posewcv_ =
        fish_->create_publisher( *this, "/pose_with_covariance", "geometry_msgs/PoseWithCovariance",
                                 1, rclcpp::PublisherOptions() );
    timer_ = this->create_wall_timer( 2s, std::bind( &AnyPublisher::timer_callback, this ) );
  }

private:
  void timer_callback()
  {
    using namespace ros_babel_fish;
    // Publish a string message
    {
      CompoundMessage::SharedPtr message = fish_->create_message_shared( "std_msgs/msg/String" );
      ( *message )["data"] = "Hello World! " + std::to_string( ++count_ );

      pub_string_->publish( *message );
    }
    // Publish a Pose
    {
      CompoundMessage::SharedPtr message = fish_->create_message_shared( "geometry_msgs/msg/Pose" );
      auto &compound = *message;
      // Different methods of assigning the values
      // Access without convenience functions
      compound["position"].as<CompoundMessage>()["x"].as<ValueMessage<double>>().setValue( 2.4 );
      // This access can be shorted using convenience methods to
      compound["position"]["y"].as<ValueMessage<double>>().setValue( 1.1 );
      // which evaluates to the same as the access above. It can be shortened even further to
      compound["position"]["z"] = 3.6;
      // where you should pay attention to the datatype. It will be casted to the actual type of the ValueMessage but
      // this may throw an exception if not possible, e.g., bool, string, time and duration can not be assigned  to a
      // different type. If it may result in a loss of information, e.g., assigning a double value to a int32 field,
      // a warning is printed and it should be avoided. If it does result in a loss of information, e.g., out of the
      // target types bounds or assigning a double value to a float ValueMessage, it will throw an exception.

      compound["orientation"]["w"] = 0.384;
      compound["orientation"]["x"] = -0.003;
      compound["orientation"]["y"] = -0.876;
      compound["orientation"]["z"] = count_ + 0.292;

      pub_pose_->publish( *message );
    }
  }

  ros_babel_fish::BabelFish::SharedPtr fish_;
  rclcpp::TimerBase::SharedPtr timer_;
  ros_babel_fish::BabelFishPublisher::SharedPtr pub_string_;
  ros_babel_fish::BabelFishPublisher::SharedPtr pub_pose_;
  ros_babel_fish::BabelFishPublisher::SharedPtr pub_posewcv_;
  size_t count_;
};

int main( int argc, char *argv[] )
{
  rclcpp::init( argc, argv );
  auto node = std::make_shared<AnyPublisher>();
  node->init();
  rclcpp::spin( node );
  rclcpp::shutdown();
  return 0;
}
