// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_BABEL_FISH_HPP
#define ROS_BABEL_FISH_BABEL_FISH_HPP

#include "ros_babel_fish/detail/babel_fish_action_client.hpp"
#include "ros_babel_fish/detail/babel_fish_action_server.hpp"
#include "ros_babel_fish/detail/babel_fish_publisher.hpp"
#include "ros_babel_fish/detail/babel_fish_service.hpp"
#include "ros_babel_fish/detail/babel_fish_service_client.hpp"
#include "ros_babel_fish/detail/babel_fish_subscription.hpp"
#include "ros_babel_fish/idl/type_support_provider.hpp"
#include "ros_babel_fish/messages/array_message.hpp"
#include "ros_babel_fish/messages/compound_message.hpp"
#include "ros_babel_fish/messages/value_message.hpp"

#include <rclcpp/node_interfaces/node_interfaces.hpp>

#include <type_traits>

namespace ros_babel_fish
{

/*!
 * Aggregate of the node interfaces required by the BabelFish factory methods.
 * Like the rclcpp factories, BabelFish does not require an rclcpp::Node. Any node-like object
 * that provides the get_node_*_interface() getters (rclcpp::Node, rclcpp_lifecycle::LifecycleNode,
 * or another rclcpp::node_interfaces::NodeInterfaces) implicitly converts to this type.
 */
using NodeInterfaces = rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeBaseInterface, rclcpp::node_interfaces::NodeClockInterface,
    rclcpp::node_interfaces::NodeGraphInterface, rclcpp::node_interfaces::NodeLoggingInterface,
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeServicesInterface, rclcpp::node_interfaces::NodeTimersInterface,
    rclcpp::node_interfaces::NodeTopicsInterface, rclcpp::node_interfaces::NodeWaitablesInterface>;

/*!
 * Allows communication using message types that are not known at compile time.
 */
class BabelFish : public std::enable_shared_from_this<BabelFish>
{

public:
  RCLCPP_SMART_PTR_DEFINITIONS( BabelFish )

  /*!
   * Constructs an instance of BabelFish with a new instance of the default description provider.
   * If you have to use multiple BabelFish instances, it is recommended to share the
   * TypeSupportProvider to prevent multiple look ups of the same message.
   */
  BabelFish();

  explicit BabelFish( std::vector<TypeSupportProvider::SharedPtr> type_support_providers );

  ~BabelFish();

  //! Wrapper for create_subscription without type.
  template<typename CallbackT>
  BabelFishSubscription::SharedPtr
  create_subscription( NodeInterfaces node, const std::string &topic, const rclcpp::QoS &qos,
                       CallbackT &&callback, rclcpp::SubscriptionOptions options = {},
                       std::chrono::nanoseconds timeout = std::chrono::nanoseconds( -1 ) )
  {
#if RCLCPP_VERSION_MAJOR >= 9
    rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> any_callback;
#else
    rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> any_callback(
        options.get_allocator() );
#endif
    any_callback.set( std::forward<CallbackT>( callback ) );
    return create_subscription( std::move( node ), topic, qos, any_callback, std::move( options ),
                                timeout );
  }

  /*!
   * This method will wait for the given topic until timeout expired (if a timeout is set) or the topic becomes available.
   * As soon as the topic is available, it will create a subscription for the topic with the passed options.
   * @param qos The quality of service options. Can be a number which will be the queue size, i.e., number of messages
   *   to queue for processing before dropping messages if the processing can't keep up.
   * @param options Subscription options. Intra-process communication is not supported. Topic statistics and
   *   QoS overrides via parameters (options.qos_overriding_options) are supported.
   * @param timeout The maximum time this call will block before returning. Set to 0 to not block at all.
   *   A negative value (the default) blocks until the topic becomes available or rclcpp shuts down.
   * @return A subscription if the topic became available before the timeout expired or a nullptr otherwise.
   *   Unlike the other factory methods, a timeout is not an error and does not throw, so callers can poll for a topic.
   *
   * @throws BabelFishException If the message type for the given topic
   *   could not be loaded or a subscription could not be created for any other reason.
   */
  BabelFishSubscription::SharedPtr
  create_subscription( NodeInterfaces node, const std::string &topic, const rclcpp::QoS &qos,
                       rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> callback,
                       rclcpp::SubscriptionOptions options = {},
                       std::chrono::nanoseconds timeout = std::chrono::nanoseconds( -1 ) );

  //! Wrapper for create_subscription using type.
  template<typename CallbackT>
  BabelFishSubscription::SharedPtr
  create_subscription( NodeInterfaces node, const std::string &topic, const std::string &type,
                       const rclcpp::QoS &qos, CallbackT &&callback,
                       rclcpp::SubscriptionOptions options = {} )
  {
#if RCLCPP_VERSION_MAJOR >= 9
    rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> any_callback;
#else
    rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> any_callback(
        options.get_allocator() );
#endif
    any_callback.set( std::forward<CallbackT>( callback ) );
    return create_subscription( std::move( node ), topic, type, qos, any_callback,
                                std::move( options ) );
  }

  /*!
   * This method will create a subscription for the given topic using the given message type.
   * Since the message type is provided, it will not wait for the topic to become available.
   * @param type The message type name for the given topic. E.g.: geometry_msgs/msg/Pose
   * @param qos The quality of service options. Can be a number which will be the queue size, i.e., number of messages
   *   to queue for processing before dropping messages if the processing can't keep up.
   * @param options Subscription options. Intra-process communication is not supported. Topic statistics and
   *   QoS overrides via parameters (options.qos_overriding_options) are supported.
   * @return A subscription to the given topic with the given message type.
   *
   * @throws BabelFishException If the given message type could not be loaded or a subscription could not
   *   be created for any other reason.
   */
  BabelFishSubscription::SharedPtr create_subscription(
      NodeInterfaces node, const std::string &topic, const std::string &type, const rclcpp::QoS &qos,
      rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> callback,
      rclcpp::SubscriptionOptions options = {} );

  /*!
   * Creates a publisher for the given topic and message type.
   * @param options Publisher options. Intra-process communication is not supported.
   *   NodeDefault silently falls back to inter-process communication.
   *   QoS overrides via parameters (options.qos_overriding_options) are supported.
   *
   * @throws BabelFishException If the type could not be loaded or the publisher could not be created for any other reason.
   */
  BabelFishPublisher::SharedPtr create_publisher( NodeInterfaces node, const std::string &topic,
                                                  const std::string &type, const rclcpp::QoS &qos,
                                                  rclcpp::PublisherOptions options = {} );

  /*!
   * Creates a service server for the given service name and type.
   * @param service_name The name under which the service should be registered.
   * @param type The type of the service, e.g., rcl_interfaces/srv/GetParameters
   * @param callback The callback that should be called when the service is called.
   * @param qos Quality of service profile for the service.
   * @param group Callback group to call the service.
   * @return A pointer to the created service server.
   *
   * @throws BabelFishException If the topic is invalid or could not be created for any other reason.
   */
  template<typename CallbackT>
  BabelFishService::SharedPtr create_service( NodeInterfaces node, const std::string &service_name,
                                              const std::string &type, CallbackT &&callback,
                                              const rclcpp::QoS &qos = rclcpp::ServicesQoS(),
                                              rclcpp::CallbackGroup::SharedPtr group = nullptr )
  {
    AnyServiceCallback any_callback( std::forward<CallbackT>( callback ) );
    return create_service( std::move( node ), service_name, type, any_callback, qos,
                           std::move( group ) );
  }

  //! @copydoc create_service
  BabelFishService::SharedPtr create_service( NodeInterfaces node, const std::string &service_name,
                                              const std::string &type, AnyServiceCallback callback,
                                              const rclcpp::QoS &qos = rclcpp::ServicesQoS(),
                                              rclcpp::CallbackGroup::SharedPtr group = nullptr );

  /*!
   * Creates a service client for the given service name and type.
   * @param service_name The name under which the service server is registered.
   * @param type The type of the service, e.g., rcl_interfaces/srv/GetParameters
   * @param qos Quality of service profile for the client.
   * @param group Callback group to handle the reply to service calls.
   * @return A service client that can be used to call the service.
   *
   * @throws BabelFishException If the topic is invalid or could not be created for any other reason.
   */
  BabelFishServiceClient::SharedPtr
  create_service_client( NodeInterfaces node, const std::string &service_name,
                         const std::string &type, const rclcpp::QoS &qos = rclcpp::ServicesQoS(),
                         rclcpp::CallbackGroup::SharedPtr group = nullptr );

  /*!
   * Creates an action server for the given name and type.
   * @param name The name under which the action server is registered.
   * @param type They type of the action.
   * @param handle_goal Callback when a new goal was received.
   * @param handle_cancel Callback when a cancel request was received.
   * @param handle_accepted Callback when a goal was accepted. Should start executing the goal.
   * @return An action server that goals can be sent to for processing.
   *
   * @throws BabelFishException If the topic is invalid or could not be created for any other reason.
   */
  BabelFishActionServer::SharedPtr create_action_server(
      NodeInterfaces node, const std::string &name, const std::string &type,
      BabelFishActionServer::GoalCallback handle_goal,
      BabelFishActionServer::CancelCallback handle_cancel,
      BabelFishActionServer::AcceptedCallback handle_accepted,
      const rcl_action_server_options_t &options = rcl_action_server_get_default_options(),
      rclcpp::CallbackGroup::SharedPtr group = nullptr );

  /*!
   * Creates an action client for the given name and type.
   * Mirrors rclcpp_action::create_client.
   * @param name The name under which the action server is registered.
   * @param type The type of the action
   * @param group The action client will be added to this callback group.
   *   If nullptr, then the action client is added to the default callback group.
   * @param options Options to pass to the underlying rcl_action_client_t.
   * @param enable_feedback_msg_optimization Enable feedback subscription content filter to
   *   optimize the handling of feedback messages. See rclcpp_action::create_client.
   * @return An action client that can be used to send goals to the action server.
   *
   * @throws BabelFishException If the topic is invalid or could not be created for any other reason.
   */
  BabelFishActionClient::SharedPtr create_action_client(
      NodeInterfaces node, const std::string &name, const std::string &type,
      rclcpp::CallbackGroup::SharedPtr group = nullptr,
      const rcl_action_client_options_t &options = rcl_action_client_get_default_options(),
      bool enable_feedback_msg_optimization = false );

  /*!
   * Creates an empty message of the given type.
   * @param type The message type, e.g.: "std_msgs/msg/Header"
   * @return An empty message of the given type
   *
   * @throws BabelFishException If the message description was not found
   */
  CompoundMessage create_message( const std::string &type ) const;

  //! @copydoc create_message
  CompoundMessage::SharedPtr create_message_shared( const std::string &type ) const;

  /*!
   * Creates a service request message for the given service type.
   * @param type The type of the service, e.g., rcl_interfaces/srv/GetParameters
   * @return An empty service request message that can be used to call a service of the given type
   *
   * @throws BabelFishException If the service description was not found
   */
  CompoundMessage create_service_request( const std::string &type ) const;

  //! @copydoc create_service_request
  CompoundMessage::SharedPtr create_service_request_shared( const std::string &type ) const;

  /*!
   * Creates an empty action goal for the given action type.
   * @param type The type of the action, e.g., example_interfaces/action/Fibonacci
   * @return An empty action goal message that can be used to send a goal to an action server.
   */
  CompoundMessage create_action_goal( const std::string &type ) const;

  //! @copydoc create_action_goal_request
  CompoundMessage::SharedPtr create_action_goal_shared( const std::string &type ) const;

  //! Loads and returns the type support for the given message type, e.g., std_msgs/msg/Int32
  //! @throws TypeSupportException If the type is invalid or could not be loaded.
  MessageTypeSupport::ConstSharedPtr get_message_type_support( const std::string &type ) const;

  //! Loads and returns the type support for the given service type, e.g., std_srvs/srv/SetBool
  //! @throws TypeSupportException If the type is invalid or could not be loaded.
  ServiceTypeSupport::ConstSharedPtr get_service_type_support( const std::string &type ) const;

  //! Loads and returns the type support for the given action type.
  //! @throws TypeSupportException If the type is invalid or could not be loaded.
  ActionTypeSupport::ConstSharedPtr get_action_type_support( const std::string &type ) const;

  std::vector<TypeSupportProvider::SharedPtr> type_support_providers();

  // ===============================================================================================
  //                             Deprecated overloads (backwards compatibility)
  // ===============================================================================================

  // The group parameter is a template so that a braced-init-list ({}) can not deduce it. This makes
  // these overloads non-viable for {} and avoids an ambiguity with the SubscriptionOptions overloads.
  template<typename CallbackT, typename GroupT,
           typename = std::enable_if_t<std::is_convertible_v<GroupT, rclcpp::CallbackGroup::SharedPtr>>>
  [[deprecated( "Pass the callback group via options.callback_group." )]]
  BabelFishSubscription::SharedPtr
  create_subscription( NodeInterfaces node, const std::string &topic, const rclcpp::QoS &qos,
                       CallbackT &&callback, GroupT &&group, rclcpp::SubscriptionOptions options = {},
                       std::chrono::nanoseconds timeout = std::chrono::nanoseconds( -1 ) )
  {
    options.callback_group = std::forward<GroupT>( group );
    return create_subscription( std::move( node ), topic, qos, std::forward<CallbackT>( callback ),
                                std::move( options ), timeout );
  }

  template<typename CallbackT, typename GroupT,
           typename = std::enable_if_t<std::is_convertible_v<GroupT, rclcpp::CallbackGroup::SharedPtr>>>
  [[deprecated( "Pass the callback group via options.callback_group." )]]
  BabelFishSubscription::SharedPtr
  create_subscription( NodeInterfaces node, const std::string &topic, const std::string &type,
                       const rclcpp::QoS &qos, CallbackT &&callback, GroupT &&group,
                       rclcpp::SubscriptionOptions options = {} )
  {
    options.callback_group = std::forward<GroupT>( group );
    return create_subscription( std::move( node ), topic, type, qos,
                                std::forward<CallbackT>( callback ), std::move( options ) );
  }

  template<typename CallbackT>
  [[deprecated( "Use the rclcpp::QoS overload." )]]
  BabelFishService::SharedPtr create_service( NodeInterfaces node, const std::string &service_name,
                                              const std::string &type, CallbackT &&callback,
                                              const rmw_qos_profile_t &qos_profile,
                                              rclcpp::CallbackGroup::SharedPtr group = nullptr )
  {
    return create_service(
        std::move( node ), service_name, type, std::forward<CallbackT>( callback ),
        rclcpp::QoS( rclcpp::QoSInitialization::from_rmw( qos_profile ), qos_profile ),
        std::move( group ) );
  }

  [[deprecated( "Use the rclcpp::QoS overload." )]]
  BabelFishServiceClient::SharedPtr
  create_service_client( NodeInterfaces node, const std::string &service_name,
                         const std::string &type, const rmw_qos_profile_t &qos_profile,
                         rclcpp::CallbackGroup::SharedPtr group = nullptr )
  {
    return create_service_client(
        std::move( node ), service_name, type,
        rclcpp::QoS( rclcpp::QoSInitialization::from_rmw( qos_profile ), qos_profile ),
        std::move( group ) );
  }

  [[deprecated( "Argument order changed to (group, options) like rclcpp_action::create_client." )]]
  BabelFishActionClient::SharedPtr
  create_action_client( NodeInterfaces node, const std::string &name, const std::string &type,
                        const rcl_action_client_options_t &options,
                        rclcpp::CallbackGroup::SharedPtr group = nullptr )
  {
    return create_action_client( std::move( node ), name, type, std::move( group ), options );
  }

private:
  std::vector<TypeSupportProvider::SharedPtr> type_support_providers_;
};
} // namespace ros_babel_fish

#endif // ROS_BABEL_FISH_BABEL_FISH_HPP
