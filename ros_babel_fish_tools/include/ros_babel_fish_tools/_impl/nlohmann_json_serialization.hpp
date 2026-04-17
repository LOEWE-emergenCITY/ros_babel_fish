// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_TOOLS_IMPL_NLOHMANN_JSON_SERIALIZATION_HPP
#define ROS_BABEL_FISH_TOOLS_IMPL_NLOHMANN_JSON_SERIALIZATION_HPP

#include "ros_babel_fish_tools/_impl/serialization_exception.hpp"
#include "ros_babel_fish_tools/_impl/serialization_helpers.hpp"
#include "ros_babel_fish_tools/nlohmann_json_serialization.hpp"
#include <ros_babel_fish/messages/value_message.hpp>
#include <ros_babel_fish/method_invoke_helpers.hpp>

#include <algorithm>
#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace ros_babel_fish_tools
{

// Forward declarations (defined below, used in compound_array serialization)
json compound_message_to_json( const ros_babel_fish::CompoundMessage &message );

namespace _impl
{

// =============================================================================
// Serialization helpers: element_to_json
// =============================================================================

template<typename T>
json element_to_json( T v )
{
  return v;
}

inline json element_to_json( long double v ) { return static_cast<double>( v ); }

inline json element_to_json( char16_t v ) { return static_cast<uint16_t>( v ); }

inline json element_to_json( const std::wstring &v ) { return wstring_to_utf8( v ); }

inline json element_to_json( const ros_babel_fish::CompoundMessage &v )
{
  return compound_message_to_json( v );
}

// =============================================================================
// Deserialization helpers: element_from_json
// =============================================================================

template<typename T>
T element_from_json( const json &j )
{
  return j.get<T>();
}

template<>
inline long double element_from_json<long double>( const json &j )
{
  return static_cast<long double>( j.get<double>() );
}

template<>
inline char16_t element_from_json<char16_t>( const json &j )
{
  return static_cast<char16_t>( j.get<uint16_t>() );
}

template<>
inline std::wstring element_from_json<std::wstring>( const json &j )
{
  return utf8_to_wstring( j.get_ref<const std::string &>() );
}

template<>
inline bool element_from_json<bool>( const json &j )
{
  if ( j.is_boolean() )
    return j.get<bool>();
  return j.get<int>() != 0;
}

// =============================================================================
// Value message serialization
// =============================================================================

inline json value_message_to_json( const ros_babel_fish::Message &msg )
{
  return ros_babel_fish::invoke_for_value_message(
      msg, []( const auto &typed ) { return element_to_json( typed.getValue() ); } );
}

// =============================================================================
// Array serialization
// =============================================================================

inline json array_message_to_json( const ros_babel_fish::ArrayMessageBase &array )
{
  return ros_babel_fish::invoke_for_array_message( array, []( const auto &typed ) {
    json arr = json::array();
    for ( size_t i = 0; i < typed.size(); ++i ) arr.push_back( element_to_json( typed[i] ) );
    return arr;
  } );
}

// =============================================================================
// Value deserialization
// =============================================================================

inline void set_value_from_json( const json &j, ros_babel_fish::Message &msg )
{
  try {
    ros_babel_fish::invoke_for_value_message( msg, [&j]( auto &typed ) {
      using T = std::decay_t<decltype( typed.getValue() )>;
      typed.setValue( element_from_json<T>( j ) );
    } );
  } catch ( const nlohmann::json::exception &e ) {
    throw SerializationException( std::string( e.what() ) );
  }
}

// Forward declaration for compound array deserialization
template<BoundsCheckBehavior Behavior>
void json_to_message( const json &j, ros_babel_fish::CompoundMessage &message );

// =============================================================================
// Array deserialization
// =============================================================================

template<BoundsCheckBehavior Behavior>
struct ArraySetters {
  template<ros_babel_fish::ArraySize ArraySize, typename ArrayType>
  void resize_array( ArrayType &typed, const json &j )
  {
    if constexpr ( ArraySize == ros_babel_fish::ArraySize::BOUNDED ||
                   ArraySize == ros_babel_fish::ArraySize::FIXED_LENGTH ) {
      if constexpr ( Behavior == BoundsCheckBehavior::Throw ) {
        if ( j.size() > typed.maxSize() )
          throw SerializationException( "array has " + std::to_string( j.size() ) +
                                        " elements but max is " + std::to_string( typed.maxSize() ) );
      }
    }
    if constexpr ( ArraySize == ros_babel_fish::ArraySize::DYNAMIC ) {
      typed.resize( j.size() );
    } else if constexpr ( ArraySize == ros_babel_fish::ArraySize::BOUNDED ) {
      typed.resize( std::min( j.size(), typed.maxSize() ) );
    }
  }

  template<ros_babel_fish::ArraySize ArraySize>
  inline void operator()( ros_babel_fish::CompoundArrayMessage_<ArraySize> &array, const json &j )
  {
    resize_array<ArraySize>( array, j );
    size_t count = std::min( j.size(), array.size() );
    for ( size_t i = 0; i < count; ++i ) {
      if ( j[i].is_null() )
        continue;
      try {
        json_to_message<Behavior>( j[i], array[i] );
      } catch ( SerializationException &e ) {
        e.prepend_index( i );
        throw;
      }
    }
  }

  template<ros_babel_fish::ArraySize ArraySize, typename ArrayT>
  inline void operator()( ros_babel_fish::ArrayMessage_<ArrayT, ArraySize> &array, const json &j )
  {
    resize_array<ArraySize>( array, j );
    size_t count = std::min( j.size(), array.size() );
    for ( size_t i = 0; i < count; ++i ) {
      if ( j[i].is_null() )
        continue;
      try {
        array.assign( i, element_from_json<ArrayT>( j[i] ) );
      } catch ( const nlohmann::json::exception &e ) {
        throw SerializationException( e.what() ).prepend_index( i );
      } catch ( SerializationException &e ) {
        e.prepend_index( i );
        throw;
      }
    }
  }
};

// =============================================================================
// Compound message deserialization
// =============================================================================

template<BoundsCheckBehavior Behavior>
void json_to_message( const json &j, ros_babel_fish::CompoundMessage &message )
{
  using namespace ros_babel_fish;
  if ( j.is_null() )
    return;
  if ( !j.is_object() )
    throw SerializationException( "Expected JSON object, got " + std::string( j.type_name() ) );

  if ( message.isTime() ) {
    try {
      builtin_interfaces::msg::Time t;
      if ( j.contains( "sec" ) && !j["sec"].is_null() )
        t.sec = j["sec"].get<int32_t>();
      else
        t.sec = int32_t( 0 );
      if ( j.contains( "nanosec" ) && !j["nanosec"].is_null() )
        t.nanosec = j["nanosec"].get<uint32_t>();
      else
        t.nanosec = uint32_t( 0 );
      message = t;
    } catch ( const nlohmann::json::exception &e ) {
      throw SerializationException( std::string( e.what() ) );
    }
    return;
  }
  if ( message.isDuration() ) {
    try {
      builtin_interfaces::msg::Duration d;
      if ( j.contains( "sec" ) && !j["sec"].is_null() )
        d.sec = j["sec"].get<int32_t>();
      else
        d.sec = int32_t( 0 );
      if ( j.contains( "nanosec" ) && !j["nanosec"].is_null() )
        d.nanosec = j["nanosec"].get<uint32_t>();
      else
        d.nanosec = uint32_t( 0 );
      message = d;
    } catch ( const nlohmann::json::exception &e ) {
      throw SerializationException( std::string( e.what() ) );
    }
    return;
  }

  auto keys = message.keys();
  for ( const auto &key : keys ) {
    if ( !j.contains( key ) || j[key].is_null() )
      continue;

    Message &child = message[key];
    const json &child_json = j[key];
    try {
      if ( child.type() == MessageTypes::Compound ) {
        json_to_message<Behavior>( child_json, child.as<CompoundMessage>() );
      } else if ( child.type() == MessageTypes::Array ) {
        if ( !child_json.is_array() )
          throw SerializationException( "Expected JSON array, got " +
                                        std::string( child_json.type_name() ) );

        ros_babel_fish::invoke_for_array_message( child.as<ArrayMessageBase>(),
                                                  ArraySetters<Behavior>{}, child_json );
      } else {
        set_value_from_json( child_json, child );
      }
    } catch ( SerializationException &e ) {
      e.prepend( key );
      throw;
    }
  }
}

} // namespace _impl

// =============================================================================
// Public API implementation
// =============================================================================

inline json message_to_json( const ros_babel_fish::Message &message )
{
  using namespace ros_babel_fish;
  if ( message.type() == MessageTypes::Compound ) {
    return compound_message_to_json( message.as<CompoundMessage>() );
  }
  if ( message.type() == MessageTypes::Array ) {
    return _impl::array_message_to_json( message.as<ArrayMessageBase>() );
  }
  return _impl::value_message_to_json( message );
}

inline json compound_message_to_json( const ros_babel_fish::CompoundMessage &message )
{
  if ( message.isTime() ) {
    const auto &t = *message.message<builtin_interfaces::msg::Time>();
    json j;
    j["sec"] = t.sec;
    j["nanosec"] = t.nanosec;
    return j;
  }
  if ( message.isDuration() ) {
    const auto &d = *message.message<builtin_interfaces::msg::Duration>();
    json j;
    j["sec"] = d.sec;
    j["nanosec"] = d.nanosec;
    return j;
  }

  json result = json::object();
  auto keys = message.keys();
  for ( const auto &key : keys ) {
    const ros_babel_fish::Message &child = message[key];
    result[key] = message_to_json( child );
  }
  return result;
}

template<BoundsCheckBehavior Behavior>
inline void json_to_message( const json &j, ros_babel_fish::CompoundMessage &message )
{
  try {
    _impl::json_to_message<Behavior>( j, message );
  } catch ( _impl::SerializationException &e ) {
    throw ros_babel_fish::BabelFishException( e.what() );
  }
}

} // namespace ros_babel_fish_tools

#endif // ROS_BABEL_FISH_TOOLS_IMPL_NLOHMANN_JSON_SERIALIZATION_HPP
