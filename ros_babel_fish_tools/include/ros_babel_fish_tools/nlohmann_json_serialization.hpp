// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_TOOLS_JSON_SERIALIZATION_HPP
#define ROS_BABEL_FISH_TOOLS_JSON_SERIALIZATION_HPP

#include "ros_babel_fish_tools/serialization_options.hpp"
#include <ros_babel_fish/babel_fish.hpp>
#include <ros_babel_fish/messages/compound_message.hpp>

// Check if nlohmann json is already included by the user, as they might want
// to use a different version than the one provided by this package.
#ifndef INCLUDE_NLOHMANN_JSON_HPP_
#include "ros_babel_fish_tools/third_party/nlohmann_json.hpp"
#endif

namespace ros_babel_fish_tools
{

using json = nlohmann::json;

/*!
 * Convert a ros_babel_fish Message to a JSON value.
 *
 * CompoundMessages become JSON objects with field names as keys.
 * ValueMessages become their corresponding JSON primitives.
 * ArrayMessages become JSON arrays.
 * builtin_interfaces/Time and Duration are serialized as objects with sec and nanosec fields.
 *
 * @param message The message to convert.
 * @return A JSON representation of the message.
 */
json message_to_json( const ros_babel_fish::Message &message );

/*!
 * Convert a ros_babel_fish CompoundMessage to a JSON object.
 *
 * @param message The compound message to convert.
 * @return A JSON object representation of the message.
 */
json compound_message_to_json( const ros_babel_fish::CompoundMessage &message );

/*!
 * Populate a ros_babel_fish CompoundMessage from a JSON object.
 *
 * Fields present in the JSON but not in the message type are ignored.
 * Fields present in the message type but not in the JSON are left at their default values.
 * For bounded and fixed-length array fields: if the JSON array is larger than the field's maximum
 * size, behaviour is controlled by the Behavior template parameter (throw by default).
 *
 * @tparam Behavior What to do when a JSON array exceeds a bounded or fixed-length field's maximum size.
 * @param j The JSON object containing the field values.
 * @param message The compound message to populate.
 * @throws ros_babel_fish::BabelFishException If a value type is incompatible or (when
 *   Behavior == BoundsCheckBehavior::Throw) a bounded or fixed-length array is exceeded.
 */
template<BoundsCheckBehavior Behavior = BoundsCheckBehavior::Throw>
void json_to_message( const json &j, ros_babel_fish::CompoundMessage &message );

} // namespace ros_babel_fish_tools

#include "ros_babel_fish_tools/_impl/nlohmann_json_serialization.hpp"

#endif // ROS_BABEL_FISH_TOOLS_JSON_SERIALIZATION_HPP
