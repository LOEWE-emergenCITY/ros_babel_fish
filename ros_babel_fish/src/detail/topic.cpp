//
// Created by Stefan Fabian on 27.07.21.
//

#include "ros_babel_fish/detail/topic.hpp"
#include "../logging.hpp"

#include <rcl/node.h>
#include <rcl/time.h>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/graph_listener.hpp>
#include <rclcpp/utilities.hpp>

#include <algorithm>
#include <map>

using namespace std::chrono_literals;

namespace ros_babel_fish
{
namespace impl
{
namespace
{
bool has_topic( TopicNodeInterfaces &node, const std::string &resolved_topic,
                std::vector<std::string> &types )
{
  const std::map<std::string, std::vector<std::string>> &topics =
      node.get_node_graph_interface()->get_topic_names_and_types();
  auto it = std::find_if( topics.begin(), topics.end(), [&resolved_topic]( const auto &entry ) {
    return entry.first == resolved_topic;
  } );
  if ( it == topics.end() )
    return false;
  types = it->second;
  return true;
}
} // namespace

bool wait_for_topic_and_type_nanoseconds( TopicNodeInterfaces node, const std::string &topic,
                                          std::vector<std::string> &types,
                                          std::chrono::nanoseconds timeout )
{
  auto start = std::chrono::steady_clock::now();
  auto node_graph = node.get_node_graph_interface();
  auto event = node_graph->get_graph_event();
  const std::string resolved_topic = resolve_topic( node, topic );
  if ( has_topic( node, resolved_topic, types ) )
    return true;
  if ( timeout == std::chrono::nanoseconds( 0 ) ) {
    // check was non-blocking, return immediately
    return false;
  }
  std::chrono::nanoseconds time_to_wait = timeout > std::chrono::nanoseconds( 0 )
                                              ? timeout - ( std::chrono::steady_clock::now() - start )
                                              : std::chrono::nanoseconds::max();
  if ( time_to_wait < std::chrono::nanoseconds( 0 ) ) {
    // check consumed entire timeout, return immediately
    return false;
  }
  do {
    if ( !rclcpp::ok( node.get_node_base_interface()->get_context() ) ) {
      return false;
    }
    // Limit each wait to 100ms to workaround an issue specific to the Connext RMW implementation.
    // A race condition means that graph changes for services becoming available may trigger the
    // wait set to wake up, but then not be reported as ready immediately after the wake up
    // (see https://github.com/ros2/rmw_connext/issues/201)
    // If no other graph events occur, the wait set will not be triggered again until the timeout
    // has been reached, despite the service being available, so we artificially limit the wait
    // time to limit the delay.
    node_graph->wait_for_graph_change(
        event, std::min( time_to_wait, std::chrono::nanoseconds( RCL_MS_TO_NS( 100 ) ) ) );
    // Because of the aforementioned race condition, we check if the topic is available even if the
    // graph event wasn't triggered.
    event->check_and_clear();
    if ( has_topic( node, resolved_topic, types ) )
      return true;

    // topic not available, wait if a timeout was specified
    if ( timeout > std::chrono::nanoseconds( 0 ) ) {
      time_to_wait = timeout - ( std::chrono::steady_clock::now() - start );
    }
    if ( std::chrono::steady_clock::now() - start > 3s ) {
      RBF2_WARN_THROTTLE(
          *node.get_node_clock_interface()->get_clock(), 3000,
          "Still waiting for topic '%s' to appear (timeout=%ld). Are you spinning the node?",
          resolved_topic.c_str(), timeout.count() );
    }
  } while ( time_to_wait > std::chrono::nanoseconds( 0 ) );
  return false; // timeout exceeded while waiting for the topic
}

bool wait_for_topic_nanoseconds( TopicNodeInterfaces node, const std::string &topic,
                                 std::chrono::nanoseconds timeout )
{
  std::vector<std::string> types;
  return wait_for_topic_and_type_nanoseconds( std::move( node ), topic, types, timeout );
}
} // namespace impl

std::string resolve_topic( TopicNodeInterfaces node, const std::string &topic )
{
  // Expands the name (~ and relative names) and applies remapping rules like rcl does when
  // creating the subscription/service.
  return node.get_node_topics_interface()->resolve_topic_name( topic );
}

std::string resolve_service_name( TopicNodeInterfaces node, const std::string &service_name )
{
  // Same as resolve_topic but with is_service=true so only service remapping rules are applied.
  auto node_base = node.get_node_base_interface();
  char *output_cstr = nullptr;
  auto allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_node_resolve_name( node_base->get_rcl_node_handle(), service_name.c_str(),
                                         allocator, true, false, &output_cstr );
  if ( ret != RCL_RET_OK ) {
    rclcpp::exceptions::throw_from_rcl_error( ret, "failed to resolve service name",
                                              rcl_get_error_state() );
  }
  std::string output( output_cstr );
  allocator.deallocate( output_cstr, allocator.state );
  return output;
}
} // namespace ros_babel_fish
