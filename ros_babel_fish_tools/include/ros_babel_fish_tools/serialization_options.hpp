// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_TOOLS_SERIALIZATION_OPTIONS_HPP
#define ROS_BABEL_FISH_TOOLS_SERIALIZATION_OPTIONS_HPP

namespace ros_babel_fish_tools
{

/*!
 * Controls how deserialization behaves when a JSON/YAML array contains more elements than the
 * bounded maximum of the target ROS array field.
 */
enum class BoundsCheckBehavior {
  Throw, ///< Throw BabelFishException if input array exceeds the bounded max size (default)
  Clamp  ///< Silently limit to max size, discarding excess elements
};

} // namespace ros_babel_fish_tools

#endif // ROS_BABEL_FISH_TOOLS_SERIALIZATION_OPTIONS_HPP
