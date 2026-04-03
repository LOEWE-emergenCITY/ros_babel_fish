// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_TOOLS_IMPL_SERIALIZATION_HELPERS_HPP
#define ROS_BABEL_FISH_TOOLS_IMPL_SERIALIZATION_HELPERS_HPP

#include <ros_babel_fish/exceptions/babel_fish_exception.hpp>
#include <ros_babel_fish/messages/array_message.hpp>

#include "ros_babel_fish_tools/_impl/third_party/utf8.h"

#include <string>
#include <type_traits>

namespace ros_babel_fish_tools
{
namespace _impl
{

// =============================================================================
// UTF-8 / wstring conversion
// =============================================================================

inline std::string wstring_to_utf8( std::wstring const &str )
{
  std::string result;
  if constexpr ( sizeof( wchar_t ) == 4 ) {
    utf8::utf32to8( str.begin(), str.end(), std::back_inserter( result ) );
  } else {
    utf8::utf16to8( str.begin(), str.end(), std::back_inserter( result ) );
  }
  return result;
}

inline std::wstring utf8_to_wstring( std::string const &str )
{
  std::wstring result;
  if constexpr ( sizeof( wchar_t ) == 4 ) {
    utf8::utf8to32( str.begin(), str.end(), std::back_inserter( result ) );
  } else {
    utf8::utf8to16( str.begin(), str.end(), std::back_inserter( result ) );
  }
  return result;
}

// =============================================================================
// Type traits shared by JSON and YAML serialization
// =============================================================================

template<typename T>
struct is_compound_array_message : std::false_type {
};
template<bool BOUNDED, bool FIXED_LENGTH>
struct is_compound_array_message<ros_babel_fish::CompoundArrayMessage_<BOUNDED, FIXED_LENGTH>>
    : std::true_type {
};

template<typename T>
struct array_element_type;
template<typename T, bool BOUNDED, bool FIXED_LENGTH>
struct array_element_type<ros_babel_fish::ArrayMessage_<T, BOUNDED, FIXED_LENGTH>> {
  using type = T;
};

} // namespace _impl
} // namespace ros_babel_fish_tools

#endif // ROS_BABEL_FISH_TOOLS_IMPL_SERIALIZATION_HELPERS_HPP
