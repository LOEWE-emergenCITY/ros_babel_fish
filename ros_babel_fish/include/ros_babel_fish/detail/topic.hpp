//
// Created by Stefan Fabian on 27.07.21.
//

#ifndef ROS_BABEL_FISH_TOPIC_HPP
#define ROS_BABEL_FISH_TOPIC_HPP

#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_graph_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>

#include <chrono>
#include <string>
#include <vector>

namespace ros_babel_fish
{

/*!
 * Node interfaces required to resolve topic names and wait for topics.
 * Any node-like object (rclcpp::Node, rclcpp_lifecycle::LifecycleNode, ...) implicitly converts to this type.
 */
using TopicNodeInterfaces = rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeBaseInterface, rclcpp::node_interfaces::NodeClockInterface,
    rclcpp::node_interfaces::NodeGraphInterface, rclcpp::node_interfaces::NodeTopicsInterface>;

namespace impl
{
bool wait_for_topic_nanoseconds( TopicNodeInterfaces node, const std::string &topic,
                                 std::chrono::nanoseconds timeout );

bool wait_for_topic_and_type_nanoseconds( TopicNodeInterfaces node, const std::string &topic,
                                          std::vector<std::string> &types,
                                          std::chrono::nanoseconds timeout );
} // namespace impl

template<typename RepT, typename PeriodT>
bool wait_for_topic(
    TopicNodeInterfaces node, const std::string &topic,
    std::chrono::duration<RepT, PeriodT> timeout = std::chrono::duration<RepT, PeriodT>( -1 ) )
{
  return impl::wait_for_topic_nanoseconds(
      std::move( node ), topic, std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) );
}

template<typename RepT, typename PeriodT>
bool wait_for_topic_and_type(
    TopicNodeInterfaces node, const std::string &topic, std::vector<std::string> &types,
    std::chrono::duration<RepT, PeriodT> timeout = std::chrono::duration<RepT, PeriodT>( -1 ) )
{
  return impl::wait_for_topic_and_type_nanoseconds(
      std::move( node ), topic, types,
      std::chrono::duration_cast<std::chrono::nanoseconds>( timeout ) );
}

/*!
 * Returns the expanded and remapped fully qualified name for the given topic name.
 * Topic and service remapping rules differ (rostopic:// vs rosservice:// rules), use
 * resolve_service_name for service names.
 */
std::string resolve_topic( TopicNodeInterfaces node, const std::string &topic );

//! Returns the expanded and remapped fully qualified name for the given service name.
std::string resolve_service_name( TopicNodeInterfaces node, const std::string &service_name );
} // namespace ros_babel_fish

#endif // ROS_BABEL_FISH_TOPIC_HPP
