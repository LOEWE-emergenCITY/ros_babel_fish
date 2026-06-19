// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_TOOLS_CLI_HPP
#define ROS_BABEL_FISH_TOOLS_CLI_HPP

#include <cstdlib>

namespace ros_babel_fish_tools
{

//! Silences the Rust-based rmw implementation (e.g. Zenoh) logging so the CLI tools' own output
//! stays clean. Must be called before rclcpp::init.
inline void silence_rmw_logging()
{
#ifdef _WIN32
  _putenv_s( "RUST_LOG", "off" );
#else
  setenv( "RUST_LOG", "off", 1 );
#endif
}

} // namespace ros_babel_fish_tools

#endif // ROS_BABEL_FISH_TOOLS_CLI_HPP
