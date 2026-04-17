// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_TOOLS_IMPL_YAML_CPP_SERIALIZATION_HPP
#define ROS_BABEL_FISH_TOOLS_IMPL_YAML_CPP_SERIALIZATION_HPP

#include "ros_babel_fish_tools/_impl/serialization_exception.hpp"
#include "ros_babel_fish_tools/_impl/serialization_helpers.hpp"
#include "ros_babel_fish_tools/yaml_cpp_serialization.hpp"
#include <ros_babel_fish/messages/value_message.hpp>
#include <ros_babel_fish/method_invoke_helpers.hpp>

#include <algorithm>
#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <limits>

namespace ros_babel_fish_tools
{

// Forward declaration (defined below, used in compound_array serialization)
YAML::Node compound_message_to_yaml( const ros_babel_fish::CompoundMessage &message );

namespace _impl
{

// =============================================================================
// Serialization helpers: element_to_yaml
// uint8_t/int8_t/unsigned char are widened to avoid yaml-cpp treating them as chars
// =============================================================================

template<typename T>
YAML::Node element_to_yaml( T v )
{
  return YAML::Node( v );
}

inline YAML::Node element_to_yaml( uint8_t v ) { return YAML::Node( static_cast<uint16_t>( v ) ); }

inline YAML::Node element_to_yaml( int8_t v ) { return YAML::Node( static_cast<int16_t>( v ) ); }

inline YAML::Node element_to_yaml( long double v )
{
  return YAML::Node( static_cast<double>( v ) );
}

inline YAML::Node element_to_yaml( char16_t v ) { return YAML::Node( static_cast<uint16_t>( v ) ); }

inline YAML::Node element_to_yaml( const std::wstring &v )
{
  return YAML::Node( wstring_to_utf8( v ) );
}

inline YAML::Node element_to_yaml( const ros_babel_fish::CompoundMessage &v )
{
  return compound_message_to_yaml( v );
}

// =============================================================================
// Deserialization helpers: element_from_yaml
// 8-bit integer types are read via their wider counterparts with a range check
// =============================================================================

template<typename T>
T element_from_yaml( const YAML::Node &n )
{
  return n.as<T>();
}

template<>
inline uint8_t element_from_yaml<uint8_t>( const YAML::Node &n )
{
  uint16_t v = n.as<uint16_t>();
  if ( v > static_cast<uint16_t>( std::numeric_limits<uint8_t>::max() ) )
    throw YAML::Exception( YAML::Mark(), "value out of range for uint8" );
  return static_cast<uint8_t>( v );
}

template<>
inline int8_t element_from_yaml<int8_t>( const YAML::Node &n )
{
  int16_t v = n.as<int16_t>();
  if ( v < static_cast<int16_t>( std::numeric_limits<int8_t>::min() ) ||
       v > static_cast<int16_t>( std::numeric_limits<int8_t>::max() ) )
    throw YAML::Exception( YAML::Mark(), "value out of range for int8" );
  return static_cast<int8_t>( v );
}

template<>
inline long double element_from_yaml<long double>( const YAML::Node &n )
{
  return static_cast<long double>( n.as<double>() );
}

template<>
inline char16_t element_from_yaml<char16_t>( const YAML::Node &n )
{
  return static_cast<char16_t>( n.as<uint16_t>() );
}

template<>
inline std::wstring element_from_yaml<std::wstring>( const YAML::Node &n )
{
  return utf8_to_wstring( n.as<std::string>() );
}

// =============================================================================
// Value message serialization
// =============================================================================

inline YAML::Node value_message_to_yaml( const ros_babel_fish::Message &msg )
{
  return ros_babel_fish::invoke_for_value_message(
      msg, []( const auto &typed ) { return element_to_yaml( typed.getValue() ); } );
}

// =============================================================================
// Array serialization
// =============================================================================

inline YAML::Node array_message_to_yaml( const ros_babel_fish::ArrayMessageBase &array )
{
  return ros_babel_fish::invoke_for_array_message( array, []( const auto &typed ) -> YAML::Node {
    YAML::Node node( YAML::NodeType::Sequence );
    for ( size_t i = 0; i < typed.size(); ++i ) node.push_back( element_to_yaml( typed[i] ) );
    return node;
  } );
}

// =============================================================================
// Value deserialization
// =============================================================================

inline void set_value_from_yaml( const YAML::Node &node, ros_babel_fish::Message &msg )
{
  try {
    ros_babel_fish::invoke_for_value_message( msg, [&node]( auto &typed ) {
      using T = std::decay_t<decltype( typed.getValue() )>;
      typed.setValue( element_from_yaml<T>( node ) );
    } );
  } catch ( const YAML::Exception &e ) {
    throw SerializationException( std::string( e.what() ) );
  }
}

// Forward declaration for compound array deserialization
template<BoundsCheckBehavior Behavior>
void yaml_to_message( const YAML::Node &node, ros_babel_fish::CompoundMessage &message );

// =============================================================================
// Array deserialization
// =============================================================================

template<BoundsCheckBehavior Behavior>
struct YamlArraySetters {
  template<bool BOUNDED, bool FIXED_LENGTH, typename ArrayType>
  void resize_array( ArrayType &typed, const YAML::Node &node )
  {
    if constexpr ( BOUNDED || FIXED_LENGTH ) {
      if constexpr ( Behavior == BoundsCheckBehavior::Throw ) {
        if ( node.size() > typed.maxSize() )
          throw SerializationException( "array has " + std::to_string( node.size() ) +
                                        " elements but max is " + std::to_string( typed.maxSize() ) );
      }
    }
    if constexpr ( !BOUNDED && !FIXED_LENGTH ) {
      typed.resize( node.size() );
    } else if constexpr ( BOUNDED ) {
      typed.resize( std::min( node.size(), typed.maxSize() ) );
    }
  }

  template<bool BOUNDED, bool FIXED_LENGTH>
  void operator()( ros_babel_fish::CompoundArrayMessage_<BOUNDED, FIXED_LENGTH> &array,
                   const YAML::Node &node )
  {
    resize_array<BOUNDED, FIXED_LENGTH>( array, node );
    size_t count = std::min( node.size(), array.size() );
    auto it = node.begin();
    for ( size_t i = 0; i < count; ++i, ++it ) {
      if ( it->IsNull() )
        continue;
      try {
        yaml_to_message<Behavior>( *it, array[i] );
      } catch ( SerializationException &e ) {
        e.prepend_index( i );
        throw;
      }
    }
  }

  template<typename ArrayT, bool BOUNDED, bool FIXED_LENGTH>
  void operator()( ros_babel_fish::ArrayMessage_<ArrayT, BOUNDED, FIXED_LENGTH> &array,
                   const YAML::Node &node )
  {
    resize_array<BOUNDED, FIXED_LENGTH>( array, node );
    size_t count = std::min( node.size(), array.size() );
    auto it = node.begin();
    for ( size_t i = 0; i < count; ++i, ++it ) {
      if ( it->IsNull() )
        continue;
      try {
        array.assign( i, element_from_yaml<ArrayT>( *it ) );
      } catch ( const YAML::Exception &e ) {
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
void yaml_to_message( const YAML::Node &node, ros_babel_fish::CompoundMessage &message )
{
  using namespace ros_babel_fish;
  if ( node.IsNull() )
    return;
  if ( !node.IsMap() )
    throw SerializationException( "expected YAML map" );

  if ( message.isTime() ) {
    try {
      builtin_interfaces::msg::Time t;
      const YAML::Node sec_node = node["sec"];
      const YAML::Node nanosec_node = node["nanosec"];
      t.sec = sec_node && !sec_node.IsNull() ? sec_node.as<int32_t>() : int32_t( 0 );
      t.nanosec =
          nanosec_node && !nanosec_node.IsNull() ? nanosec_node.as<uint32_t>() : uint32_t( 0 );
      message = t;
    } catch ( const YAML::Exception &e ) {
      throw SerializationException( std::string( e.what() ) );
    }
    return;
  }
  if ( message.isDuration() ) {
    try {
      builtin_interfaces::msg::Duration d;
      const YAML::Node sec_node = node["sec"];
      const YAML::Node nanosec_node = node["nanosec"];
      d.sec = sec_node && !sec_node.IsNull() ? sec_node.as<int32_t>() : int32_t( 0 );
      d.nanosec =
          nanosec_node && !nanosec_node.IsNull() ? nanosec_node.as<uint32_t>() : uint32_t( 0 );
      message = d;
    } catch ( const YAML::Exception &e ) {
      throw SerializationException( std::string( e.what() ) );
    }
    return;
  }

  auto keys = message.keys();
  for ( const auto &key : keys ) {
    const YAML::Node &child_node = node[key];
    if ( !child_node || child_node.IsNull() )
      continue;

    Message &child = message[key];
    try {
      if ( child.type() == MessageTypes::Compound ) {
        yaml_to_message<Behavior>( child_node, child.as<CompoundMessage>() );
      } else if ( child.type() == MessageTypes::Array ) {
        if ( !child_node.IsSequence() )
          throw SerializationException( "expected YAML sequence" );

        ros_babel_fish::invoke_for_array_message( child.as<ArrayMessageBase>(),
                                                  YamlArraySetters<Behavior>{}, child_node );
      } else {
        set_value_from_yaml( child_node, child );
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

inline YAML::Node message_to_yaml( const ros_babel_fish::Message &message )
{
  using namespace ros_babel_fish;
  if ( message.type() == MessageTypes::Compound ) {
    return compound_message_to_yaml( message.as<CompoundMessage>() );
  }
  if ( message.type() == MessageTypes::Array ) {
    return _impl::array_message_to_yaml( message.as<ArrayMessageBase>() );
  }
  return _impl::value_message_to_yaml( message );
}

inline YAML::Node compound_message_to_yaml( const ros_babel_fish::CompoundMessage &message )
{
  if ( message.isTime() ) {
    const auto &t = *message.message<builtin_interfaces::msg::Time>();
    YAML::Node node;
    node["sec"] = t.sec;
    node["nanosec"] = t.nanosec;
    return node;
  }
  if ( message.isDuration() ) {
    const auto &d = *message.message<builtin_interfaces::msg::Duration>();
    YAML::Node node;
    node["sec"] = d.sec;
    node["nanosec"] = d.nanosec;
    return node;
  }

  YAML::Node node( YAML::NodeType::Map );
  auto keys = message.keys();
  for ( const auto &key : keys ) { node[key] = message_to_yaml( message[key] ); }
  return node;
}

template<BoundsCheckBehavior Behavior>
inline void yaml_to_message( const YAML::Node &node, ros_babel_fish::CompoundMessage &message )
{
  try {
    _impl::yaml_to_message<Behavior>( node, message );
  } catch ( _impl::SerializationException &e ) {
    throw ros_babel_fish::BabelFishException( e.what() );
  }
}

} // namespace ros_babel_fish_tools

#endif // ROS_BABEL_FISH_TOOLS_IMPL_YAML_CPP_SERIALIZATION_HPP
