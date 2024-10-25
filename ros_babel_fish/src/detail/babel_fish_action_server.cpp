// Copyright (c) 2024 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "../logging.hpp"

#include "ros_babel_fish/detail/babel_fish_action_server.hpp"
#include "ros_babel_fish/idl/serialization.hpp"
#include "ros_babel_fish/messages/array_message.hpp"

using CompoundMessage = ros_babel_fish::CompoundMessage;
using UUIDMessage = ros_babel_fish::FixedLengthArrayMessage<rclcpp_action::GoalUUID::value_type>;

namespace rclcpp_action
{

ServerGoalHandle<ros_babel_fish::impl::BabelFishAction>::ServerGoalHandle(
    ros_babel_fish::ActionTypeSupport::ConstSharedPtr type_support,
    std::shared_ptr<rcl_action_goal_handle_t> rcl_handle, GoalUUID uuid,
    std::shared_ptr<const ActionT::Goal> goal,
    std::function<void( const GoalUUID &, std::shared_ptr<void> )> on_terminal_state,
    std::function<void( const GoalUUID & )> on_executing,
    std::function<void( std::shared_ptr<void> )> publish_feedback )
    : ServerGoalHandleBase( std::move( rcl_handle ) ), type_support_( std::move( type_support ) ),
      goal_( std::move( goal ) ), uuid_( uuid ),
      on_terminal_state_( std::move( on_terminal_state ) ),
      on_executing_( std::move( on_executing ) ), publish_feedback_( std::move( publish_feedback ) )
{
}

void ServerGoalHandle<ros_babel_fish::impl::BabelFishAction>::publish_feedback(
    const ActionT::Feedback &feedback_msg ) const
{
  auto feedback_message = CompoundMessage( *type_support_->feedback_message_type_support );
  feedback_message["goal_id"]["uuid"].as<UUIDMessage>() = uuid_;
  feedback_message["feedback"].as<CompoundMessage>() = feedback_msg;
  publish_feedback_( feedback_message.type_erased_message() );
}

void ServerGoalHandle<ros_babel_fish::impl::BabelFishAction>::abort( const ActionT::Result &result_msg )
{
  _abort();
  auto response = CompoundMessage( type_support_->result_service_type_support->response() );
  response["status"] = action_msgs::msg::GoalStatus::STATUS_ABORTED;
  response["result"].as<CompoundMessage>() = result_msg;
  on_terminal_state_( uuid_, response.type_erased_message() );
}

void ServerGoalHandle<ros_babel_fish::impl::BabelFishAction>::succeed( const ActionT::Result &result_msg )
{
  _succeed();
  auto response = CompoundMessage( type_support_->result_service_type_support->response() );
  response["status"] = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;
  response["result"].as<CompoundMessage>() = result_msg;
  on_terminal_state_( uuid_, response.type_erased_message() );
}

void ServerGoalHandle<ros_babel_fish::impl::BabelFishAction>::canceled( const ActionT::Result &result_msg )
{
  _canceled();
  auto response = CompoundMessage( type_support_->result_service_type_support->response() );
  response["status"] = action_msgs::msg::GoalStatus::STATUS_CANCELED;
  response["result"].as<CompoundMessage>() = result_msg;
  on_terminal_state_( uuid_, response.type_erased_message() );
}

void ServerGoalHandle<ros_babel_fish::impl::BabelFishAction>::execute()
{
  _execute();
  on_executing_( uuid_ );
}

CompoundMessage ServerGoalHandle<ros_babel_fish::impl::BabelFishAction>::create_result_message() const
{
  auto response_type_support = type_support_->result_service_type_support->response();
  // Find result in response message
  size_t i;
  for ( i = 0; i < response_type_support->member_count_; ++i ) {
    if ( response_type_support.getMember( i )->name_ == std::string( "result" ) )
      break;
  }
  if ( i == response_type_support->member_count_ ) {
    throw ros_babel_fish::BabelFishException(
        "Could not create action result message: Result response message does not contain a "
        "'result' field" );
  }
  auto msg = ros_babel_fish::createContainer( response_type_support.getMember( i ) );
  return { response_type_support.getMember( i ), msg };
}

CompoundMessage ServerGoalHandle<ros_babel_fish::impl::BabelFishAction>::create_feedback_message() const
{
  auto feedback_type_support =
      ros_babel_fish::MessageMembersIntrospection( *type_support_->feedback_message_type_support );
  // Find feedback in FeedbackMessage
  size_t i;
  for ( i = 0; i < feedback_type_support->member_count_; ++i ) {
    if ( feedback_type_support.getMember( i )->name_ == std::string( "feedback" ) )
      break;
  }
  if ( i == feedback_type_support->member_count_ ) {
    throw ros_babel_fish::BabelFishException(
        "Could not create action feedback message: FeedbackMessage does not contain a 'feedback' "
        "field" );
  }
  auto msg = ros_babel_fish::createContainer( feedback_type_support.getMember( i ) );
  return { feedback_type_support.getMember( i ), msg };
}

ServerGoalHandle<ros_babel_fish::impl::BabelFishAction>::~ServerGoalHandle()
{
  // Cancel goal if handle was allowed to destruct without reaching a terminal state
  if ( try_canceling() ) {
    auto null_result =
        CompoundMessage::make_shared( type_support_->result_service_type_support->response() );
    ( *null_result )["status"] = action_msgs::msg::GoalStatus::STATUS_CANCELED;
    on_terminal_state_( uuid_, null_result );
  }
}

// =================================================================================================
//                                            Server
// =================================================================================================

Server<ros_babel_fish::impl::BabelFishAction>::Server(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging, const std::string &name,
    ros_babel_fish::ActionTypeSupport::ConstSharedPtr type_support,
    const rcl_action_server_options_t &options,
    Server<ros_babel_fish::impl::BabelFishAction>::GoalCallback handle_goal,
    Server<ros_babel_fish::impl::BabelFishAction>::CancelCallback handle_cancel,
    Server<ros_babel_fish::impl::BabelFishAction>::AcceptedCallback handle_accepted )
    : ServerBase( std::move( node_base ), std::move( node_clock ), std::move( node_logging ), name,
                  &type_support->type_support_handle, options ),
      type_support_( std::move( type_support ) ), handle_goal_( std::move( handle_goal ) ),
      handle_cancel_( std::move( handle_cancel ) ), handle_accepted_( std::move( handle_accepted ) )
{
}

Server<ros_babel_fish::impl::BabelFishAction>::~Server() = default;

std::pair<GoalResponse, std::shared_ptr<void>>
Server<ros_babel_fish::impl::BabelFishAction>::call_handle_goal_callback( GoalUUID &uuid,
                                                                          std::shared_ptr<void> message )
{
  const auto &request_type_support = type_support_->goal_service_type_support->request();
  auto request = CompoundMessage::make_shared( request_type_support, message );
  auto goal = std::shared_ptr<ActionT::Goal>( request, &( *request )["goal"].as<CompoundMessage>() );
  GoalResponse user_response = handle_goal_( uuid, goal );

  const auto &response_type_support = type_support_->goal_service_type_support->response();
  auto response = CompoundMessage( response_type_support );
  response["accepted"] = GoalResponse::ACCEPT_AND_EXECUTE == user_response ||
                         GoalResponse::ACCEPT_AND_DEFER == user_response;
  return std::make_pair( user_response, response.type_erased_message() );
}

CancelResponse
Server<ros_babel_fish::impl::BabelFishAction>::call_handle_cancel_callback( const GoalUUID &uuid )
{
  std::shared_ptr<ServerGoalHandle<ActionT>> goal_handle;
  std::unique_lock lock( goal_handles_mutex_ );
  if ( auto element = goal_handles_.find( uuid ); element != goal_handles_.end() ) {
    goal_handle = element->second.lock();
  }
  lock.unlock();

  CancelResponse response = CancelResponse::REJECT;
  if ( goal_handle ) {
    response = handle_cancel_( goal_handle );
    if ( CancelResponse::ACCEPT == response ) {
      try {
        goal_handle->_cancel_goal();
      } catch ( const rclcpp::exceptions::RCLError &ex ) {
        RBF2_DEBUG( "Failed to cancel goal in call_handle_cancel_callback: %s", ex.what() );
        return CancelResponse::REJECT;
      }
    }
  }
  return response;
}

void Server<ros_babel_fish::impl::BabelFishAction>::call_goal_accepted_callback(
    std::shared_ptr<rcl_action_goal_handle_t> rcl_goal_handle, GoalUUID uuid,
    std::shared_ptr<void> goal_request_message )
{
  std::shared_ptr<ServerGoalHandle<ActionT>> goal_handle;
  std::weak_ptr<Server<ActionT>> weak_this = this->shared_from_this();

  std::function<void( const GoalUUID &, std::shared_ptr<void> )> on_terminal_state =
      [weak_this]( const GoalUUID &goal_uuid, std::shared_ptr<void> result_message ) {
        std::shared_ptr<Server<ActionT>> shared_this = weak_this.lock();
        if ( !shared_this ) {
          return;
        }
        // Send result message to anyone that asked
        shared_this->publish_result( goal_uuid, std::move( result_message ) );
        // Publish a status message any time a goal handle changes state
        shared_this->publish_status();
        // notify base so it can recalculate the expired goal timer
        shared_this->notify_goal_terminal_state();
        // Delete data now (ServerBase and rcl_action_server_t keep data until goal handle expires)
        std::lock_guard lock( shared_this->goal_handles_mutex_ );
        shared_this->goal_handles_.erase( goal_uuid );
      };

  std::function<void( const GoalUUID & )> on_executing = [weak_this]( const GoalUUID &goal_uuid ) {
    std::shared_ptr<Server<ActionT>> shared_this = weak_this.lock();
    if ( !shared_this ) {
      return;
    }
    (void)goal_uuid;
    // Publish a status message any time a goal handle changes state
    shared_this->publish_status();
  };

  std::function<void( std::shared_ptr<void> )> publish_feedback =
      [weak_this]( std::shared_ptr<void> feedback_msg ) {
        std::shared_ptr<Server<ActionT>> shared_this = weak_this.lock();
        if ( !shared_this ) {
          return;
        }
        shared_this->publish_feedback( std::move( feedback_msg ) );
      };

  const auto &request_type_support = type_support_->goal_service_type_support->request();
  auto request = CompoundMessage::make_shared( request_type_support, goal_request_message );
  auto goal =
      std::shared_ptr<const ActionT::Goal>( request, &( *request )["goal"].as<CompoundMessage>() );
  goal_handle.reset( new ServerGoalHandle<ActionT>( type_support_, rcl_goal_handle, uuid, goal,
                                                    on_terminal_state, on_executing,
                                                    publish_feedback ) );
  std::unique_lock lock( goal_handles_mutex_ );
  goal_handles_[uuid] = goal_handle;
  lock.unlock();
  handle_accepted_( goal_handle );
}

GoalUUID Server<ros_babel_fish::impl::BabelFishAction>::get_goal_id_from_goal_request( void *message )
{
  GoalUUID result;
  auto compound = CompoundMessage(
      type_support_->goal_service_type_support->request(),
      std::shared_ptr<void>( message, []( const void * ) { /* not ours to delete */ } ) );
  const auto &uuid = compound["goal_id"]["uuid"].as<UUIDMessage>();
  for ( size_t i = 0; i < uuid.size(); ++i ) { result[i] = uuid[i]; }
  return result;
}

std::shared_ptr<void> Server<ros_babel_fish::impl::BabelFishAction>::create_goal_request()
{
  return ros_babel_fish::createContainer( type_support_->goal_service_type_support->request() );
}

GoalUUID Server<ros_babel_fish::impl::BabelFishAction>::get_goal_id_from_result_request( void *message )
{
  GoalUUID result;
  auto compound = CompoundMessage(
      type_support_->result_service_type_support->request(),
      std::shared_ptr<void>( message, []( const void * ) { /* not ours to delete */ } ) );
  const auto &uuid = compound["goal_id"]["uuid"].as<UUIDMessage>();
  for ( size_t i = 0; i < uuid.size(); ++i ) { result[i] = uuid[i]; }
  return result;
}

std::shared_ptr<void> Server<ros_babel_fish::impl::BabelFishAction>::create_result_request()
{
  return ros_babel_fish::createContainer( type_support_->result_service_type_support->request() );
}

std::shared_ptr<void> Server<ros_babel_fish::impl::BabelFishAction>::create_result_response(
    decltype( action_msgs::msg::GoalStatus::status ) status )
{
  auto message =
      ros_babel_fish::createContainer( type_support_->result_service_type_support->response() );
  auto result = CompoundMessage( type_support_->result_service_type_support->response(), message );
  result["status"] = status;
  return std::static_pointer_cast<void>( message );
}

} // namespace rclcpp_action
