// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_BABEL_FISH_EXCEPTION_HPP
#define ROS_BABEL_FISH_BABEL_FISH_EXCEPTION_HPP

#include <rclcpp/exceptions.hpp>

namespace ros_babel_fish
{

class BabelFishException : public std::runtime_error
{
public:
  explicit BabelFishException( const std::string &msg ) : std::runtime_error( msg ) { }
};
} // namespace ros_babel_fish

#endif // ROS_BABEL_FISH_BABEL_FISH_EXCEPTION_HPP
