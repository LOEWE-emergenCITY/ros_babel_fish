// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros2_babel_fish/detail/babel_fish_publisher.hpp"

#include <rclcpp/detail/resolve_use_intra_process.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/serialized_message.hpp>

namespace ros2_babel_fish
{

BabelFishPublisher::BabelFishPublisher( rclcpp::node_interfaces::NodeBaseInterface *node_base,
                                        const rosidl_message_type_support_t &type_support,
                                        const std::string &topic, const rclcpp::QoS &qos,
                                        const rclcpp::PublisherOptions &options )
    : PublisherBase( node_base, topic, type_support,
                     options.to_rcl_publisher_options<CompoundMessage>( qos ),
                     options.event_callbacks, options.use_default_callbacks ),
      options_( options )
{
  // Setup continues in the post construction method, post_init_setup().
}

void BabelFishPublisher::post_init_setup(
    rclcpp::node_interfaces::NodeBaseInterface *node_base, const std::string &topic,
    const rclcpp::QoS &qos,
    const rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> &options )
{
  // Here intra process publishing would be set up if someone cares to add support
  (void)node_base;
  (void)topic;
  (void)options;
  (void)qos;
  (void)options;
}

void BabelFishPublisher::publish( std::unique_ptr<CompoundMessage> msg )
{
  this->do_inter_process_publish( *msg );
}

void BabelFishPublisher::publish( const CompoundMessage &msg )
{
  // Intra process is currently not supported.
  return this->do_inter_process_publish( msg );
}

void BabelFishPublisher::publish( const rcl_serialized_message_t &serialized_msg )
{
  return this->do_serialized_publish( &serialized_msg );
}

void BabelFishPublisher::publish( const rclcpp::SerializedMessage &serialized_msg )
{
  return this->do_serialized_publish( &serialized_msg.get_rcl_serialized_message() );
}

void BabelFishPublisher::do_inter_process_publish( const CompoundMessage &msg )
{
  auto status = rcl_publish( publisher_handle_.get(), msg.type_erased_message().get(), nullptr );

  if ( RCL_RET_PUBLISHER_INVALID == status ) {
    rcl_reset_error(); // next call will reset error message if not context
    if ( rcl_publisher_is_valid_except_context( publisher_handle_.get() ) ) {
      rcl_context_t *context = rcl_publisher_get_context( publisher_handle_.get() );
      if ( nullptr != context && !rcl_context_is_valid( context ) ) {
        // publisher is invalid due to context being shutdown
        return;
      }
    }
  }
  if ( RCL_RET_OK != status ) {
    rclcpp::exceptions::throw_from_rcl_error( status, "failed to publish message" );
  }
}

void BabelFishPublisher::do_serialized_publish( const rcl_serialized_message_t *serialized_msg )
{
  auto status = rcl_publish_serialized_message( publisher_handle_.get(), serialized_msg, nullptr );
  if ( RCL_RET_OK != status ) {
    rclcpp::exceptions::throw_from_rcl_error( status, "failed to publish serialized message" );
  }
}
} // namespace ros2_babel_fish
