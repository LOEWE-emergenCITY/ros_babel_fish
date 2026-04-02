// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_TOOLS_IMPL_NLOHMANN_JSON_SERIALIZATION_HPP
#define ROS_BABEL_FISH_TOOLS_IMPL_NLOHMANN_JSON_SERIALIZATION_HPP

#include "ros_babel_fish_tools/_impl/serialization_exception.hpp"
#include "ros_babel_fish_tools/_impl/serialization_helpers.hpp"
#include "ros_babel_fish_tools/nlohmann_json_serialization.hpp"
#include <ros_babel_fish/messages/value_message.hpp>
#include <ros_babel_fish/method_invoke_helpers.hpp>

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
{ return v; }

inline json element_to_json( long double v ) { return static_cast<double>( v ); }

inline json element_to_json( char16_t v ) { return static_cast<uint16_t>( v ); }

inline json element_to_json( const std::wstring &v ) { return wstring_to_utf8( v ); }

inline json element_to_json( const ros_babel_fish::CompoundMessage &v )
{ return compound_message_to_json( v ); }

// =============================================================================
// Deserialization helpers: element_from_json
// =============================================================================

template<typename T>
T element_from_json( const json &j )
{ return j.get<T>(); }

template<>
inline long double element_from_json<long double>( const json &j )
{ return static_cast<long double>( j.get<double>() ); }

template<>
inline char16_t element_from_json<char16_t>( const json &j )
{ return static_cast<char16_t>( j.get<uint16_t>() ); }

template<>
inline std::wstring element_from_json<std::wstring>( const json &j )
{ return utf8_to_wstring( j.get_ref<const std::string &>() ); }

// =============================================================================
// Value message serialization
// =============================================================================

inline json value_message_to_json( const ros_babel_fish::Message &msg )
{
  using namespace ros_babel_fish;
  switch ( msg.type() ) {
  case MessageTypes::Bool:
    return element_to_json( msg.value<bool>() );
  case MessageTypes::Octet:
  case MessageTypes::UInt8:
    return element_to_json( msg.value<uint8_t>() );
  case MessageTypes::UInt16:
    return element_to_json( msg.value<uint16_t>() );
  case MessageTypes::UInt32:
    return element_to_json( msg.value<uint32_t>() );
  case MessageTypes::UInt64:
    return element_to_json( msg.value<uint64_t>() );
  case MessageTypes::Int8:
    return element_to_json( msg.value<int8_t>() );
  case MessageTypes::Int16:
    return element_to_json( msg.value<int16_t>() );
  case MessageTypes::Int32:
    return element_to_json( msg.value<int32_t>() );
  case MessageTypes::Int64:
    return element_to_json( msg.value<int64_t>() );
  case MessageTypes::Float:
    return element_to_json( msg.value<float>() );
  case MessageTypes::Double:
    return element_to_json( msg.value<double>() );
  case MessageTypes::LongDouble:
    return element_to_json( msg.value<long double>() );
  case MessageTypes::Char:
    return element_to_json( msg.value<uint8_t>() );
  case MessageTypes::WChar:
    return element_to_json( msg.value<char16_t>() );
  case MessageTypes::String:
    return element_to_json( msg.value<std::string>() );
  case MessageTypes::WString:
    return element_to_json( msg.value<std::wstring>() );
  default:
    return nullptr;
  }
}

// =============================================================================
// Array serialization
// =============================================================================

inline json array_message_to_json( const ros_babel_fish::ArrayMessageBase &array )
{
  return ros_babel_fish::invoke_for_array_message( array, []( const auto &typed ) -> json {
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
  using namespace ros_babel_fish;
  try {
    switch ( msg.type() ) {
    case MessageTypes::Bool:
      if ( j.is_boolean() )
        msg = j.get<bool>();
      else
        msg = j.get<int>() != 0;
      break;
    case MessageTypes::Octet:
    case MessageTypes::UInt8:
      msg = j.get<uint8_t>();
      break;
    case MessageTypes::UInt16:
      msg = j.get<uint16_t>();
      break;
    case MessageTypes::UInt32:
      msg = j.get<uint32_t>();
      break;
    case MessageTypes::UInt64:
      msg = j.get<uint64_t>();
      break;
    case MessageTypes::Int8:
      msg = j.get<int8_t>();
      break;
    case MessageTypes::Int16:
      msg = j.get<int16_t>();
      break;
    case MessageTypes::Int32:
      msg = j.get<int32_t>();
      break;
    case MessageTypes::Int64:
      msg = j.get<int64_t>();
      break;
    case MessageTypes::Float:
      msg = j.get<float>();
      break;
    case MessageTypes::Double:
      msg = j.get<double>();
      break;
    case MessageTypes::LongDouble:
      msg = element_from_json<long double>( j );
      break;
    case MessageTypes::Char:
      msg = j.get<uint8_t>();
      break;
    case MessageTypes::WChar:
      msg = element_from_json<char16_t>( j );
      break;
    case MessageTypes::String:
      msg = j.get<std::string>();
      break;
    case MessageTypes::WString:
      msg = element_from_json<std::wstring>( j );
      break;
    default:
      break;
    }
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
inline void set_array_from_json( const json &j, ros_babel_fish::ArrayMessageBase &array )
{
  ros_babel_fish::invoke_for_array_message( array, [&]( auto &typed ) {
    using ArrayT = std::remove_cv_t<std::remove_reference_t<decltype( typed )>>;

    if ( !typed.isFixedSize() ) {
      if constexpr ( Behavior == BoundsCheckBehavior::Throw ) {
        if ( typed.isBounded() && j.size() > typed.maxSize() )
          throw SerializationException( "array has " + std::to_string( j.size() ) +
                                        " elements but max is " + std::to_string( typed.maxSize() ) );
      }
      typed.resize( typed.isBounded() ? std::min( j.size(), typed.maxSize() ) : j.size() );
    }
    size_t count = std::min( j.size(), typed.size() );

    if constexpr ( is_compound_array_message<ArrayT>::value ) {
      for ( size_t i = 0; i < count; ++i ) {
        if ( j[i].is_null() )
          continue;
        try {
          json_to_message<Behavior>( j[i], typed[i] );
        } catch ( SerializationException &e ) {
          e.prepend_index( i );
          throw;
        }
      }
    } else {
      using T = typename array_element_type<ArrayT>::type;
      for ( size_t i = 0; i < count; ++i ) {
        if ( j[i].is_null() )
          continue;
        try {
          typed.assign( i, element_from_json<T>( j[i] ) );
        } catch ( const nlohmann::json::exception &e ) {
          SerializationException ex( e.what() );
          ex.prepend_index( i );
          throw ex;
        } catch ( SerializationException &e ) {
          e.prepend_index( i );
          throw;
        }
      }
    }
  } );
}

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
    throw SerializationException( "expected JSON object, got " + std::string( j.type_name() ) );

  if ( message.isTime() ) {
    builtin_interfaces::msg::Time t;
    t.sec = j.value( "sec", int32_t( 0 ) );
    t.nanosec = j.value( "nanosec", uint32_t( 0 ) );
    message = t;
    return;
  }
  if ( message.isDuration() ) {
    builtin_interfaces::msg::Duration d;
    d.sec = j.value( "sec", int32_t( 0 ) );
    d.nanosec = j.value( "nanosec", uint32_t( 0 ) );
    message = d;
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
          throw SerializationException( "expected JSON array, got " +
                                        std::string( child_json.type_name() ) );
        set_array_from_json<Behavior>( child_json, child.as<ArrayMessageBase>() );
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
