// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_BABEL_FISH_ACTION_SERVER_HPP
#define ROS_BABEL_FISH_BABEL_FISH_ACTION_SERVER_HPP

#include "ros_babel_fish/detail/babel_fish_action.hpp"
#include <rclcpp_action/server.hpp>
#include <ros_babel_fish/idl/type_support.hpp>
#include <ros_babel_fish/messages/compound_message.hpp>

namespace rclcpp_action
{

template<>
class ServerGoalHandle<ros_babel_fish::impl::BabelFishAction>
    : public ServerGoalHandleBase,
      public std::enable_shared_from_this<ServerGoalHandle<ros_babel_fish::impl::BabelFishAction>>
{
public:
  using ActionT = ros_babel_fish::impl::BabelFishAction;

  /// @see ServerGoalHandle<ActionT>::publish_feedback
  void publish_feedback( const ActionT::Feedback &feedback_msg ) const;

  /// @see ServerGoalHandle<ActionT>::abort
  void abort( const ActionT::Result &result_msg );

  /// @see ServerGoalHandle<ActionT>::succeed
  void succeed( const ActionT::Result &result_msg );

  // @see ServerGoalHandle<ActionT>::canceled
  void canceled( const ActionT::Result &result_msg );

  /// @see ServerGoalHandle<ActionT>::execute
  void execute();

  /// Get the user provided message describing the goal.
  const std::shared_ptr<const ActionT::Goal> &get_goal() const { return goal_; }

  /// Get the unique identifier of the goal
  const GoalUUID &get_goal_id() const { return uuid_; }

  ~ServerGoalHandle() override;

  ActionT::Result create_result_message() const;

  ActionT::Feedback create_feedback_message() const;

private:
  ServerGoalHandle( ros_babel_fish::ActionTypeSupport::ConstSharedPtr type_support,
                    std::shared_ptr<rcl_action_goal_handle_t> rcl_handle, GoalUUID uuid,
                    std::shared_ptr<const typename ActionT::Goal> goal,
                    std::function<void( const GoalUUID &, std::shared_ptr<void> )> on_terminal_state,
                    std::function<void( const GoalUUID & )> on_executing,
                    std::function<void( std::shared_ptr<void> )> publish_feedback );

  ros_babel_fish::ActionTypeSupport::ConstSharedPtr type_support_;
  const std::shared_ptr<const ActionT::Goal> goal_;
  const GoalUUID uuid_;

  friend class Server<ActionT>;

  std::function<void( const GoalUUID &, std::shared_ptr<void> )> on_terminal_state_;
  std::function<void( const GoalUUID & )> on_executing_;
  std::function<void( std::shared_ptr<void> )> publish_feedback_;
};

template<>
class Server<ros_babel_fish::impl::BabelFishAction>
    : public ServerBase,
      public std::enable_shared_from_this<Server<ros_babel_fish::impl::BabelFishAction>>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE( Server )
  using ActionT = ros_babel_fish::impl::BabelFishAction;
  using GoalHandle = ServerGoalHandle<ActionT>;

  using GoalCallback =
      std::function<GoalResponse( const GoalUUID &, std::shared_ptr<const ActionT::Goal> )>;

  using CancelCallback =
      std::function<CancelResponse( const std::shared_ptr<ServerGoalHandle<ActionT>> )>;

  using AcceptedCallback = std::function<void( std::shared_ptr<ServerGoalHandle<ActionT>> )>;

  //! Do not call directly, this is private API and might change. Use BabelFish::create_action_server.
  Server( rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
          rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
          rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
          const std::string &name, ros_babel_fish::ActionTypeSupport::ConstSharedPtr type_support,
          const rcl_action_server_options_t &options, GoalCallback handle_goal,
          CancelCallback handle_cancel, AcceptedCallback handle_accepted );

  ~Server() override;

private:
  std::pair<GoalResponse, std::shared_ptr<void>>
  call_handle_goal_callback( GoalUUID &uuid, std::shared_ptr<void> message ) override;

  CancelResponse call_handle_cancel_callback( const GoalUUID &uuid ) override;

  void call_goal_accepted_callback( std::shared_ptr<rcl_action_goal_handle_t> rcl_goal_handle,
                                    GoalUUID uuid,
                                    std::shared_ptr<void> goal_request_message ) override;

  GoalUUID get_goal_id_from_goal_request( void *message ) override;

  std::shared_ptr<void> create_goal_request() override;

  GoalUUID get_goal_id_from_result_request( void *message ) override;

  std::shared_ptr<void> create_result_request() override;

  std::shared_ptr<void>
  create_result_response( decltype( action_msgs::msg::GoalStatus::status ) status ) override;

  ros_babel_fish::ActionTypeSupport::ConstSharedPtr type_support_;

  GoalCallback handle_goal_;
  CancelCallback handle_cancel_;
  AcceptedCallback handle_accepted_;

  using GoalHandleWeakPtr = std::weak_ptr<ServerGoalHandle<ActionT>>;
  /// A map of goal id to goal handle weak pointers.
  /// This is used to provide a goal handle to handle_cancel.
  std::unordered_map<GoalUUID, GoalHandleWeakPtr> goal_handles_;
  std::mutex goal_handles_mutex_;
};
} // namespace rclcpp_action

namespace ros_babel_fish
{
using BabelFishActionServer = rclcpp_action::Server<impl::BabelFishAction>;
using BabelFishActionServerGoalHandle = rclcpp_action::ServerGoalHandle<impl::BabelFishAction>;
} // namespace ros_babel_fish

#endif // ROS_BABEL_FISH_BABEL_FISH_ACTION_HPP
