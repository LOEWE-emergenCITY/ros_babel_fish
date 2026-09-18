// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/babel_fish.hpp"

#include "logging.hpp"
#include "ros_babel_fish/detail/babel_fish_service_client.hpp"
#include "ros_babel_fish/detail/topic.hpp"
#include "ros_babel_fish/exceptions/babel_fish_exception.hpp"
#include "ros_babel_fish/idl/providers/local_type_support_provider.hpp"

#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/detail/qos_parameters.hpp>
#include <rclcpp/detail/resolve_enable_topic_statistics.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/topic_statistics/subscription_topic_statistics.hpp>
#include <rclcpp/wait_set.hpp>
#include <statistics_msgs/msg/metrics_message.hpp>

namespace ros_babel_fish
{

namespace
{

/*!
 * Creates the topic statistics collector, publisher and publish timer for a subscription if topic statistics
 * are enabled in the options (or by node default). Mirrors rclcpp::detail::create_subscription.
 * @return The topic statistics instance or nullptr if topic statistics are disabled.
 */
std::shared_ptr<rclcpp::topic_statistics::SubscriptionTopicStatistics>
create_subscription_topic_statistics( NodeInterfaces &node,
                                      const rclcpp::SubscriptionOptions &options )
{
  auto node_base = node.get_node_base_interface();
  if ( !rclcpp::detail::resolve_enable_topic_statistics( options, *node_base ) )
    return nullptr;

  if ( options.topic_stats_options.publish_period <= std::chrono::milliseconds( 0 ) ) {
    throw std::invalid_argument(
        "topic_stats_options.publish_period must be greater than 0, specified value of " +
        std::to_string( options.topic_stats_options.publish_period.count() ) + " ms" );
  }

  auto node_parameters = node.get_node_parameters_interface();
  auto node_topics = node.get_node_topics_interface();
  auto publisher = rclcpp::create_publisher<statistics_msgs::msg::MetricsMessage>(
      node_parameters, node_topics, options.topic_stats_options.publish_topic,
      options.topic_stats_options.qos );

  auto subscription_topic_stats =
      std::make_shared<rclcpp::topic_statistics::SubscriptionTopicStatistics>( node_base->get_name(),
                                                                               publisher );

  std::weak_ptr<rclcpp::topic_statistics::SubscriptionTopicStatistics> weak_subscription_topic_stats(
      subscription_topic_stats );
  auto publish_callback = [weak_subscription_topic_stats]() {
    if ( auto stats = weak_subscription_topic_stats.lock(); stats != nullptr ) {
      stats->publish_message_and_reset_measurements();
    }
  };

  auto timer = rclcpp::create_wall_timer( std::chrono::duration_cast<std::chrono::nanoseconds>(
                                              options.topic_stats_options.publish_period ),
                                          publish_callback, options.callback_group, node_base.get(),
                                          node.get_node_timers_interface().get() );

  subscription_topic_stats->set_publisher_timer( timer );
  return subscription_topic_stats;
}

/*!
 * Applies QoS overrides from parameters if requested via options.qos_overriding_options.
 * Mirrors rclcpp::create_publisher / rclcpp::create_subscription which declare
 * qos_overrides.<topic>.<entity>.* parameters and use their values instead of the default qos.
 */
template<typename EntityQosParametersTraits>
rclcpp::QoS resolve_qos_overrides( NodeInterfaces &node,
                                   const rclcpp::QosOverridingOptions &qos_overriding_options,
                                   const std::string &topic, const rclcpp::QoS &default_qos,
                                   EntityQosParametersTraits traits )
{
  if ( qos_overriding_options.get_policy_kinds().empty() )
    return default_qos;
  auto node_parameters = node.get_node_parameters_interface();
  return rclcpp::detail::declare_qos_parameters(
      qos_overriding_options, node_parameters,
      node.get_node_topics_interface()->resolve_topic_name( topic ), default_qos, traits );
}

void check_intra_process_setting( rclcpp::IntraProcessSetting setting, const char *entity )
{
  // Intra-process communication is not supported for type-erased messages.
  // NodeDefault silently falls back to inter-process, an explicit request is ignored with a warning.
  if ( setting == rclcpp::IntraProcessSetting::Enable ) {
    RBF2_WARN( "%s requested intra-process communication which is not supported by ros_babel_fish. "
               "Falling back to inter-process communication.",
               entity );
  }
}
} // namespace

BabelFish::BabelFish()
{
  type_support_providers_.push_back( std::make_shared<LocalTypeSupportProvider>() );
}

BabelFish::BabelFish( std::vector<TypeSupportProvider::SharedPtr> type_support_providers )
    : type_support_providers_( std::move( type_support_providers ) )
{
}

BabelFish::~BabelFish() = default;

BabelFishSubscription::SharedPtr BabelFish::create_subscription(
    NodeInterfaces node, const std::string &topic, const rclcpp::QoS &qos,
    rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> callback,
    rclcpp::SubscriptionOptions options, std::chrono::nanoseconds timeout )
{
  const std::string resolved_topic = resolve_topic( node, topic );
  std::vector<std::string> types;
  if ( !wait_for_topic_and_type( node, topic, types, timeout ) )
    return nullptr;
  if ( types.empty() ) {
    RBF2_ERROR( "Could not subscribe to '%s'.Topic is available but has no type!",
                resolved_topic.c_str() );
    return nullptr;
  }

  if ( types.size() > 1 ) {
    RBF2_INFO( "Topic '%s' has more than one type. Selecting the first arbitrarily: '%s'.",
               resolved_topic.c_str(), types[0].c_str() );
  }

  return create_subscription( std::move( node ), topic, types[0], qos, std::move( callback ),
                              std::move( options ) );
}

BabelFishSubscription::SharedPtr BabelFish::create_subscription(
    NodeInterfaces node, const std::string &topic, const std::string &type, const rclcpp::QoS &qos,
    rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> callback,
    rclcpp::SubscriptionOptions options )
{
  check_intra_process_setting( options.use_intra_process_comm, "Subscription" );
  // Not supported by ROS2 for serialized messages, make sure the node default doesn't enable it.
  options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  MessageTypeSupport::ConstSharedPtr type_support = get_message_type_support( type );
  if ( type_support == nullptr ) {
    throw BabelFishException( "Failed to create a subscriber for type: " + type +
                              ". Type not found!" );
  }
  try {
    auto subscription_topic_stats = create_subscription_topic_statistics( node, options );
    const rclcpp::QoS actual_qos =
        resolve_qos_overrides( node, options.qos_overriding_options, topic, qos,
                               rclcpp::detail::SubscriptionQosParametersTraits{} );
    auto subscription = std::make_shared<BabelFishSubscription>(
        node.get_node_base_interface().get(), type_support, topic, actual_qos,
        std::move( callback ), options, std::move( subscription_topic_stats ) );

    node.get_node_topics_interface()->add_subscription( subscription, options.callback_group );
    return subscription;
  } catch ( const std::exception &ex ) {
    throw BabelFishException( "Failed to create Subscription: " + std::string( ex.what() ) );
  }
}

BabelFishPublisher::SharedPtr
BabelFish::create_publisher( NodeInterfaces node, const std::string &topic, const std::string &type,
                             const rclcpp::QoS &qos, rclcpp::PublisherOptions options )
{
  auto node_topics = node.get_node_topics_interface();

  check_intra_process_setting( options.use_intra_process_comm, "Publisher" );
  // Not supported by ROS2 for serialized messages, make sure the node default doesn't enable it.
  options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  MessageTypeSupport::ConstSharedPtr type_support = get_message_type_support( type );
  if ( type_support == nullptr ) {
    throw BabelFishException( "Failed to create a publisher for type: " + type + ". Type not found!" );
  }
  const rclcpp::QoS actual_qos =
      resolve_qos_overrides( node, options.qos_overriding_options, topic, qos,
                             rclcpp::detail::PublisherQosParametersTraits{} );
  auto result = BabelFishPublisher::make_shared( node.get_node_base_interface().get(),
                                                 type_support->type_support_handle, topic,
                                                 actual_qos, options );
  result->post_init_setup( node.get_node_base_interface().get(), topic, actual_qos, options );
  // Add the publisher to the node topics interface.
  node_topics->add_publisher( result, options.callback_group );
  return result;
}

BabelFishService::SharedPtr
BabelFish::create_service( NodeInterfaces node, const std::string &service_name,
                           const std::string &type, AnyServiceCallback callback,
                           const rclcpp::QoS &qos, rclcpp::CallbackGroup::SharedPtr group )
{
  ServiceTypeSupport::ConstSharedPtr type_support = get_service_type_support( type );
  if ( type_support == nullptr ) {
    throw BabelFishException( "Failed to create a service for type: " + type + ". Type not found!" );
  }
  rcl_service_options_t options = rcl_service_get_default_options();
  options.qos = qos.get_rmw_qos_profile();
  auto result =
      BabelFishService::make_shared( node.get_node_base_interface()->get_shared_rcl_node_handle(),
                                     service_name, type_support, std::move( callback ), options );
  node.get_node_services_interface()->add_service( result, std::move( group ) );
  return result;
}

BabelFishServiceClient::SharedPtr
BabelFish::create_service_client( NodeInterfaces node, const std::string &service_name,
                                  const std::string &type, const rclcpp::QoS &qos,
                                  rclcpp::CallbackGroup::SharedPtr group )
{
  ServiceTypeSupport::ConstSharedPtr type_support = get_service_type_support( type );
  if ( type_support == nullptr ) {
    throw BabelFishException( "Failed to create a service client for type: " + type +
                              ". Type not found!" );
  }
  rcl_client_options_t options = rcl_client_get_default_options();
  options.qos = qos.get_rmw_qos_profile();
  try {
    auto result = BabelFishServiceClient::make_shared( node.get_node_base_interface().get(),
                                                       node.get_node_graph_interface(),
                                                       service_name, type_support, options );
    node.get_node_services_interface()->add_client( result, std::move( group ) );
    return result;
  } catch ( const std::exception &ex ) {
    throw BabelFishException( "Failed to create Service Client: " + std::string( ex.what() ) );
  }
}

BabelFishActionServer::SharedPtr BabelFish::create_action_server(
    NodeInterfaces node, const std::string &name, const std::string &type,
    BabelFishActionServer::GoalCallback handle_goal,
    BabelFishActionServer::CancelCallback handle_cancel,
    BabelFishActionServer::AcceptedCallback handle_accepted,
    const rcl_action_server_options_t &options, rclcpp::CallbackGroup::SharedPtr group )
{
  ActionTypeSupport::ConstSharedPtr type_support = get_action_type_support( type );
  if ( type_support == nullptr ) {
    throw BabelFishException( "Failed to create an action server for type: " + type +
                              ". Type not found!" );
  }
  std::weak_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> weak_node =
      node.get_node_waitables_interface();
  std::weak_ptr<rclcpp::CallbackGroup> weak_group = group;
  bool group_is_null = ( nullptr == group.get() );

  auto deleter = [weak_node, weak_group, group_is_null]( BabelFishActionServer *ptr ) {
    if ( nullptr == ptr ) {
      return;
    }
    if ( auto shared_node = weak_node.lock(); shared_node != nullptr ) {
      // API expects a shared pointer, give it one with a deleter that does nothing.
      std::shared_ptr<BabelFishActionServer> fake_shared_ptr(
          ptr, []( const BabelFishActionServer * ) { /* do nothing */ } );

      if ( group_is_null ) {
        // Was added to default group
        shared_node->remove_waitable( fake_shared_ptr, nullptr );
      } else {
        // Was added to a specific group
        auto shared_group = weak_group.lock();
        if ( shared_group ) {
          shared_node->remove_waitable( fake_shared_ptr, shared_group );
        }
      }
    }
    delete ptr;
  };
  try {
    std::shared_ptr<BabelFishActionServer> server(
        new BabelFishActionServer( node.get_node_base_interface(), node.get_node_clock_interface(),
                                   node.get_node_logging_interface(), name, type_support, options,
                                   std::move( handle_goal ), std::move( handle_cancel ),
                                   std::move( handle_accepted ) ),
        deleter );
    node.get_node_waitables_interface()->add_waitable( server, std::move( group ) );
    return server;
  } catch ( const std::exception &ex ) {
    throw BabelFishException( "Failed to create Action Server: " + std::string( ex.what() ) );
  }
}

BabelFishActionClient::SharedPtr
BabelFish::create_action_client( NodeInterfaces node, const std::string &name,
                                 const std::string &type, rclcpp::CallbackGroup::SharedPtr group,
                                 const rcl_action_client_options_t &options,
                                 bool enable_feedback_msg_optimization )
{
  ActionTypeSupport::ConstSharedPtr type_support = get_action_type_support( type );
  if ( type_support == nullptr ) {
    throw BabelFishException( "Failed to create an action client for type: " + type +
                              ". Type not found!" );
  }
  std::weak_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> weak_node =
      node.get_node_waitables_interface();
  std::weak_ptr<rclcpp::CallbackGroup> weak_group = group;
  bool group_is_null = ( nullptr == group.get() );

  auto deleter = [weak_node, weak_group, group_is_null]( BabelFishActionClient *ptr ) {
    if ( nullptr == ptr ) {
      return;
    }
    if ( auto shared_node = weak_node.lock(); shared_node != nullptr ) {
      // API expects a shared pointer, give it one with a deleter that does nothing.
      std::shared_ptr<BabelFishActionClient> fake_shared_ptr(
          ptr, []( const BabelFishActionClient * ) { /* not ours to delete */ } );

      if ( group_is_null ) {
        // Was added to default group
        shared_node->remove_waitable( fake_shared_ptr, nullptr );
      } else {
        // Was added to a specific group
        auto shared_group = weak_group.lock();
        if ( shared_group ) {
          shared_node->remove_waitable( fake_shared_ptr, shared_group );
        }
      }
    }
    delete ptr;
  };

  try {
    std::shared_ptr<BabelFishActionClient> action_client(
        new BabelFishActionClient( node.get_node_base_interface(), node.get_node_graph_interface(),
                                   node.get_node_logging_interface(), name, type_support, options,
                                   enable_feedback_msg_optimization ),
        deleter );

    node.get_node_waitables_interface()->add_waitable( action_client, std::move( group ) );
    return action_client;
  } catch ( const std::exception &ex ) {
    throw BabelFishException( "Failed to create Action Client: " + std::string( ex.what() ) );
  }
}

CompoundMessage BabelFish::create_message( const std::string &type ) const
{
  MessageTypeSupport::ConstSharedPtr type_support = get_message_type_support( type );
  if ( type_support == nullptr ) {
    throw BabelFishException( "BabelFish doesn't know a message of type: " + type );
  }
  return CompoundMessage( *type_support );
}

CompoundMessage::SharedPtr BabelFish::create_message_shared( const std::string &type ) const
{
  MessageTypeSupport::ConstSharedPtr type_support = get_message_type_support( type );
  if ( type_support == nullptr ) {
    throw BabelFishException( "BabelFish doesn't know a message of type: " + type );
  }
  return CompoundMessage::make_shared( *type_support );
}

CompoundMessage BabelFish::create_service_request( const std::string &type ) const
{
  const ServiceTypeSupport::ConstSharedPtr &type_support = get_service_type_support( type );
  if ( type_support == nullptr ) {
    throw BabelFishException( "BabelFish doesn't know a service of type: " + type );
  }
  return CompoundMessage( type_support->request() );
}

CompoundMessage::SharedPtr BabelFish::create_service_request_shared( const std::string &type ) const
{
  const ServiceTypeSupport::ConstSharedPtr &type_support = get_service_type_support( type );
  if ( type_support == nullptr ) {
    throw BabelFishException( "BabelFish doesn't know a service of type: " + type );
  }
  return CompoundMessage::make_shared( type_support->request() );
}

CompoundMessage BabelFish::create_action_goal( const std::string &type ) const
{
  const ActionTypeSupport::ConstSharedPtr &type_support = get_action_type_support( type );
  if ( type_support == nullptr ) {
    throw BabelFishException( "BabelFish doesn't know an action of type: " + type );
  }
  MessageMembersIntrospection introspection = type_support->goal_service_type_support->request();
  size_t index =
      std::find_if( introspection->members_, introspection->members_ + introspection->member_count_,
                    []( const auto &a ) { return std::strcmp( a.name_, "goal" ) == 0; } ) -
      introspection->members_;
  return CompoundMessage( type_support->goal_service_type_support->request().getMember( index ) );
}

CompoundMessage::SharedPtr BabelFish::create_action_goal_shared( const std::string &type ) const
{
  return CompoundMessage::make_shared( create_action_goal( type ) );
}

MessageTypeSupport::ConstSharedPtr BabelFish::get_message_type_support( const std::string &type ) const
{
  for ( const auto &provider : type_support_providers_ ) {
    MessageTypeSupport::ConstSharedPtr result = provider->getMessageTypeSupport( type );
    if ( result == nullptr )
      continue;
    return result;
  }
  return nullptr;
}

ServiceTypeSupport::ConstSharedPtr BabelFish::get_service_type_support( const std::string &type ) const
{
  for ( const auto &provider : type_support_providers_ ) {
    ServiceTypeSupport::ConstSharedPtr result = provider->getServiceTypeSupport( type );
    if ( result == nullptr )
      continue;
    return result;
  }
  return nullptr;
}

ActionTypeSupport::ConstSharedPtr BabelFish::get_action_type_support( const std::string &type ) const
{
  for ( const auto &provider : type_support_providers_ ) {
    ActionTypeSupport::ConstSharedPtr result = provider->getActionTypeSupport( type );
    if ( result == nullptr )
      continue;
    return result;
  }
  return nullptr;
}

std::vector<TypeSupportProvider::SharedPtr> BabelFish::type_support_providers()
{
  return type_support_providers_;
}
} // namespace ros_babel_fish
