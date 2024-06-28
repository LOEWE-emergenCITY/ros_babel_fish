//
// Created by Stefan Fabian on 23.07.21.
//

#include <benchmark/benchmark.h>
#include <ros_babel_fish/babel_fish.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

static void PoseStamped( benchmark::State &state )
{
  auto data = std::make_shared<geometry_msgs::msg::PoseStamped>();
  data->header.stamp = rclcpp::Time( 42 );
  data->header.frame_id = "test_frame";
  data->pose.position.x = 1;
  data->pose.position.y = 0;
  data->pose.position.z = 3;
  data->pose.orientation.w = 1;
  auto void_data = std::static_pointer_cast<void>( data );
  ros_babel_fish::BabelFish fish;
  using ros_babel_fish::MessageTypeSupport;
  MessageTypeSupport::ConstSharedPtr type_support =
      fish.get_message_type_support( "geometry_msgs/msg/PoseStamped" );
  for ( auto _ : state ) {
    ros_babel_fish::CompoundMessage cm( *type_support, void_data );
    if ( cm["header"]["stamp"].value<rclcpp::Time>() != rclcpp::Time( 42L, RCL_ROS_TIME ) )
      throw std::runtime_error( "stamp" );
    if ( cm["header"]["frame_id"].value<std::string>() != "test_frame" )
      throw std::runtime_error( "frame" );
    if ( cm["pose"]["position"]["x"].value<double>() != 1 )
      throw std::runtime_error( "pos.x" );
    if ( cm["pose"]["position"]["y"].value<double>() != 0 )
      throw std::runtime_error( "pos.y" );
    if ( cm["pose"]["position"]["z"].value<double>() != 3 )
      throw std::runtime_error( "pos.z" );
    if ( cm["pose"]["orientation"]["z"].value<double>() != 0 )
      throw std::runtime_error( "ori.z" );
    if ( cm["pose"]["orientation"]["w"].value<double>() != 1 )
      throw std::runtime_error( "ori.w" );
  }
}

BENCHMARK( PoseStamped );

BENCHMARK_MAIN();
