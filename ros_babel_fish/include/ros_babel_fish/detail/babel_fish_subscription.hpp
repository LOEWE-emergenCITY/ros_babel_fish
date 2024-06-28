// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_BABEL_FISH_SUBSCRIPTION_HPP
#define ROS_BABEL_FISH_BABEL_FISH_SUBSCRIPTION_HPP

#include <ros_babel_fish/messages/compound_message.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/timer.hpp>

namespace ros_babel_fish
{
class BabelFish;

class BabelFishSubscription : public rclcpp::SubscriptionBase
{
private:
  using SubscriptionTopicStatisticsSharedPtr =
      std::shared_ptr<rclcpp::topic_statistics::SubscriptionTopicStatistics>;

public:
  RCLCPP_SMART_PTR_DEFINITIONS( BabelFishSubscription )

  BabelFishSubscription(
      rclcpp::node_interfaces::NodeBaseInterface *node,
      MessageTypeSupport::ConstSharedPtr type_support, const std::string &topic_name,
      const rclcpp::QoS &qos,
      rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> callback,
      const rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> &options,
      SubscriptionTopicStatisticsSharedPtr subscription_topic_statistics = nullptr );

  ~BabelFishSubscription() override;

  std::shared_ptr<void> create_message() override;

  std::shared_ptr<rclcpp::SerializedMessage> create_serialized_message() override;

  void handle_message( std::shared_ptr<void> &message,
                       const rclcpp::MessageInfo &message_info ) override;

  void handle_serialized_message( const std::shared_ptr<rclcpp::SerializedMessage> &serialized_message,
                                  const rclcpp::MessageInfo &message_info ) override;

  void handle_loaned_message( void *loaned_message, const rclcpp::MessageInfo &message_info ) override;

  void return_message( std::shared_ptr<void> &message ) override;

  void return_serialized_message( std::shared_ptr<rclcpp::SerializedMessage> &message ) override;

  bool take( CompoundMessage &message_out, rclcpp::MessageInfo &message_info_out );

  MessageTypeSupport::ConstSharedPtr get_message_type_support() const;

  std::string get_message_type() const;

  rclcpp::dynamic_typesupport::DynamicMessageType::SharedPtr get_shared_dynamic_message_type() override;

  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr get_shared_dynamic_message() override;

  rclcpp::dynamic_typesupport::DynamicSerializationSupport::SharedPtr
  get_shared_dynamic_serialization_support() override;

  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr create_dynamic_message() override;

  void
  return_dynamic_message( rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr &message ) override;

  void handle_dynamic_message( const rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr &message,
                               const rclcpp::MessageInfo &message_info ) override;

private:
  RCLCPP_DISABLE_COPY( BabelFishSubscription )

  MessageTypeSupport::ConstSharedPtr type_support_;
  rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> callback_;
  /// Component which computes and publishes topic statistics for this subscriber
  SubscriptionTopicStatisticsSharedPtr subscription_topic_statistics_{ nullptr };
};
} // namespace ros_babel_fish

#endif // ROS_BABEL_FISH_BABEL_FISH_SUBSCRIPTION_HPP
