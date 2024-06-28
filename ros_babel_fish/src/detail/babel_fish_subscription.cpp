//
// Created by stefan on 22.01.21.
//

#include "ros_babel_fish/detail/babel_fish_subscription.hpp"
#include "../logging.hpp"
#include "ros_babel_fish/idl/serialization.hpp"

#include <rcl/rcl.h>
#include <rclcpp/node.hpp>

namespace ros_babel_fish
{

BabelFishSubscription::BabelFishSubscription(
    rclcpp::node_interfaces::NodeBaseInterface *node_base,
    MessageTypeSupport::ConstSharedPtr type_support, const std::string &topic_name,
    const rclcpp::QoS &qos,
    rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> callback,
    const rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> &options,
    SubscriptionTopicStatisticsSharedPtr subscription_topic_statistics )
    : rclcpp::SubscriptionBase( node_base, type_support->type_support_handle, topic_name,
                                options.to_rcl_subscription_options( qos ), options.event_callbacks,
                                options.use_default_callbacks,
                                callback.is_serialized_message_callback()
                                    ? rclcpp::DeliveredMessageKind::SERIALIZED_MESSAGE
                                    : rclcpp::DeliveredMessageKind::ROS_MESSAGE ),
      type_support_( std::move( type_support ) ), callback_( callback )
{
  // TODO: Adding intra process support [very low priority]

  if ( subscription_topic_statistics != nullptr ) {
    this->subscription_topic_statistics_ = std::move( subscription_topic_statistics );
  }

  TRACEPOINT( rclcpp_subscription_init, static_cast<const void *>( get_subscription_handle().get() ),
              static_cast<const void *>( this ) );
  TRACEPOINT( rclcpp_subscription_callback_added, static_cast<const void *>( this ),
              static_cast<const void *>( &callback_ ) );
  // The callback object gets copied, so if registration is done too early/before this point
  // (e.g. in `AnySubscriptionCallback::set()`), its address won't match any address used later
  // in subsequent tracepoints.
#ifndef TRACETOOLS_DISABLED
  callback_.register_callback_for_tracing();
#endif
  RBF2_DEBUG_STREAM( "Subscribed to: " << topic_name );
}

BabelFishSubscription::~BabelFishSubscription()
{
  RBF2_DEBUG_STREAM( "Unsubscribed from: " << get_topic_name() );
}

std::shared_ptr<void> BabelFishSubscription::create_message()
{
  return createContainer( *type_support_ );
}

std::shared_ptr<rclcpp::SerializedMessage> BabelFishSubscription::create_serialized_message()
{
  return std::make_shared<rclcpp::SerializedMessage>( 0 );
}

void BabelFishSubscription::handle_message( std::shared_ptr<void> &message,
                                            const rclcpp::MessageInfo &message_info )
{
  std::chrono::time_point<std::chrono::system_clock> now;
  if ( subscription_topic_statistics_ ) {
    // get current time before executing callback to
    // exclude callback duration from topic statistics result.
    now = std::chrono::system_clock::now();
  }

  callback_.dispatch( CompoundMessage::make_shared( *type_support_, message ), message_info );

  if ( subscription_topic_statistics_ ) {
    const auto nanos = std::chrono::time_point_cast<std::chrono::nanoseconds>( now );
    const auto time = rclcpp::Time( nanos.time_since_epoch().count() );
    subscription_topic_statistics_->handle_message( message_info.get_rmw_message_info(), time );
  }
}

void BabelFishSubscription::handle_serialized_message(
    const std::shared_ptr<rclcpp::SerializedMessage> &serialized_message,
    const rclcpp::MessageInfo &message_info )
{
  std::chrono::time_point<std::chrono::system_clock> now;
  if ( subscription_topic_statistics_ ) {
    // get current time before executing callback to
    // exclude callback duration from topic statistics result.
    now = std::chrono::system_clock::now();
  }

  callback_.dispatch( serialized_message, message_info );

  if ( subscription_topic_statistics_ ) {
    const auto nanos = std::chrono::time_point_cast<std::chrono::nanoseconds>( now );
    const auto time = rclcpp::Time( nanos.time_since_epoch().count() );
    subscription_topic_statistics_->handle_message( message_info.get_rmw_message_info(), time );
  }
}

void BabelFishSubscription::handle_loaned_message( void *loaned_message,
                                                   const rclcpp::MessageInfo &message_info )
{
  // Not handled
  (void)loaned_message;
  (void)message_info;
}

void BabelFishSubscription::return_message( std::shared_ptr<void> &message )
{
  auto typed_message = std::static_pointer_cast<rclcpp::SerializedMessage>( message );
  return_serialized_message( typed_message );
}

void BabelFishSubscription::return_serialized_message( std::shared_ptr<rclcpp::SerializedMessage> &message )
{
  message.reset();
}

bool BabelFishSubscription::take( CompoundMessage &message_out, rclcpp::MessageInfo &info_out )
{
  std::shared_ptr<void> type_erased = create_message();
  if ( type_erased == nullptr || !take_type_erased( type_erased.get(), info_out ) )
    return false;
  message_out = CompoundMessage( *type_support_, std::move( type_erased ) );
  return true;
}

MessageTypeSupport::ConstSharedPtr BabelFishSubscription::get_message_type_support() const
{
  return type_support_;
}

std::string BabelFishSubscription::get_message_type() const { return type_support_->name; }

rclcpp::dynamic_typesupport::DynamicMessageType::SharedPtr
BabelFishSubscription::get_shared_dynamic_message_type()
{
  throw rclcpp::exceptions::UnimplementedError(
      "get_shared_dynamic_message_type is not implemented for BabelFishSubscription" );
}

rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr
BabelFishSubscription::get_shared_dynamic_message()
{
  throw rclcpp::exceptions::UnimplementedError(
      "get_shared_dynamic_message is not implemented for BabelFishSubscription" );
}

rclcpp::dynamic_typesupport::DynamicSerializationSupport::SharedPtr
BabelFishSubscription::get_shared_dynamic_serialization_support()
{
  throw rclcpp::exceptions::UnimplementedError(
      "get_shared_dynamic_serialization_support is not implemented for BabelFishSubscription" );
}

rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr BabelFishSubscription::create_dynamic_message()
{
  throw rclcpp::exceptions::UnimplementedError(
      "create_dynamic_message is not implemented for BabelFishSubscription" );
}

void BabelFishSubscription::return_dynamic_message(
    rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr & )
{
  throw rclcpp::exceptions::UnimplementedError(
      "return_dynamic_message is not implemented for BabelFishSubscription" );
}

void BabelFishSubscription::handle_dynamic_message(
    const rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr &, const rclcpp::MessageInfo & )
{
  throw rclcpp::exceptions::UnimplementedError(
      "handle_dynamic_message is not implemented for BabelFishSubscription" );
}
} // namespace ros_babel_fish
