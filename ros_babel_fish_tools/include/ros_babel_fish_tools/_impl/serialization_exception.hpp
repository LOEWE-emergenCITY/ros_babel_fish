// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_TOOLS_IMPL_SERIALIZATION_EXCEPTION_HPP
#define ROS_BABEL_FISH_TOOLS_IMPL_SERIALIZATION_EXCEPTION_HPP

#include <ros_babel_fish/exceptions/babel_fish_exception.hpp>

#include <string>

namespace ros_babel_fish_tools
{
namespace _impl
{

/*!
 * Internal exception used during JSON/YAML deserialization.
 * The path to the failing field accumulates as the exception bubbles up through recursive calls.
 * Each call site catches, prepends its key or index, and re-throws.
 * At the public API boundary this is caught and converted to a BabelFishException.
 */
struct SerializationException : ros_babel_fish::BabelFishException {
  explicit SerializationException( std::string msg )
      : BabelFishException( msg ), leaf_msg( std::move( msg ) )
  {
  }

  SerializationException( const SerializationException &other ) noexcept
      : BabelFishException( other ), path( other.path ), leaf_msg( other.leaf_msg )
  {
  }

  const char *what() const noexcept override
  {
    full_ = path.empty() ? leaf_msg : "at '" + path + "': " + leaf_msg;
    return full_.c_str();
  }

  SerializationException &prepend( const std::string &segment )
  {
    path = path.empty() ? segment : segment + "." + path;
    return *this;
  }

  SerializationException &prepend_index( size_t i )
  {
    std::string idx = "[" + std::to_string( i ) + "]";
    path = path.empty() ? idx : idx + "." + path;
    return *this;
  }

private:
  std::string path;
  std::string leaf_msg;
  mutable std::string full_;
};

} // namespace _impl
} // namespace ros_babel_fish_tools

#endif // ROS_BABEL_FISH_TOOLS_IMPL_SERIALIZATION_EXCEPTION_HPP
