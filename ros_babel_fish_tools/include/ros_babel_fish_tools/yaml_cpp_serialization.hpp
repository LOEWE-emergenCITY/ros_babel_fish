// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_TOOLS_YAML_CPP_SERIALIZATION_HPP
#define ROS_BABEL_FISH_TOOLS_YAML_CPP_SERIALIZATION_HPP

#include "ros_babel_fish_tools/serialization_options.hpp"
#include <ros_babel_fish/babel_fish.hpp>
#include <ros_babel_fish/messages/compound_message.hpp>

#include <yaml-cpp/yaml.h>

namespace ros_babel_fish_tools
{

/*!
 * Convert a ros_babel_fish Message to a YAML node.
 *
 * CompoundMessages become YAML maps.
 * ValueMessages become their corresponding YAML primitives.
 * ArrayMessages become YAML sequences.
 * builtin_interfaces/Time and Duration are serialized as maps with sec and nanosec fields.
 *
 * @param message The message to convert.
 * @return A YAML representation of the message.
 */
YAML::Node message_to_yaml( const ros_babel_fish::Message &message );

/*!
 * Convert a ros_babel_fish CompoundMessage to a YAML map.
 *
 * @param message The compound message to convert.
 * @return A YAML map representation of the message.
 */
YAML::Node compound_message_to_yaml( const ros_babel_fish::CompoundMessage &message );

/*!
 * Populate a ros_babel_fish CompoundMessage from a YAML node.
 *
 * Fields present in the YAML but not in the message type are ignored.
 * Fields present in the message type but not in the YAML are left at their default values.
 * For bounded and fixed-length array fields: if the YAML sequence is larger than the field's
 * maximum size, behaviour is controlled by the Behavior template parameter (throw by default).
 *
 * @tparam Behavior What to do when a YAML sequence exceeds a bounded or fixed-length field's maximum size.
 * @param node The YAML map containing the field values.
 * @param message The compound message to populate.
 * @throws ros_babel_fish::BabelFishException If a value type is incompatible or (when
 *   Behavior == BoundsCheckBehavior::Throw) a bounded or fixed-length array is exceeded.
 */
template<BoundsCheckBehavior Behavior = BoundsCheckBehavior::Throw>
void yaml_to_message( const YAML::Node &node, ros_babel_fish::CompoundMessage &message );

} // namespace ros_babel_fish_tools

#include "ros_babel_fish_tools/_impl/yaml_cpp_serialization.hpp"

#endif // ROS_BABEL_FISH_TOOLS_YAML_CPP_SERIALIZATION_HPP
