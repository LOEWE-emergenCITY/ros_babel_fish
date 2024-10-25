// Copyright (c) 2024 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_BABEL_FISH_ACTION_HPP
#define ROS_BABEL_FISH_BABEL_FISH_ACTION_HPP

#include "ros_babel_fish/messages/compound_message.hpp"

namespace ros_babel_fish
{
struct ActionTypeSupport;

namespace impl
{
struct BabelFishAction {
  using Feedback = CompoundMessage;
  using Goal = CompoundMessage;
  using Result = CompoundMessage;

  struct Impl {
    struct SendGoalService {
      using Request = CompoundMessage;
      using Response = CompoundMessage;
    };
    struct GetResultService {
      using Request = CompoundMessage;
      using Response = CompoundMessage;
    };
    using FeedbackMessage = CompoundMessage;
    struct CancelGoalService {
      using Request = CompoundMessage;
      using Response = CompoundMessage;
    };
    using GoalStatusMessage = CompoundMessage;
  };
};
} // namespace impl
} // namespace ros_babel_fish

#endif // ROS_BABEL_FISH_BABEL_FISH_ACTION_HPP
