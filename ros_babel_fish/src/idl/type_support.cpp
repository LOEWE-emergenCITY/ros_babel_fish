// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "ros_babel_fish/idl/type_support.hpp"

#include <assert.h>
#include <regex>

namespace ros_babel_fish
{

MessageMemberIntrospection MessageMembersIntrospection::getMember( size_t index ) const
{
  assert( index < value->member_count_ );
  return MessageMemberIntrospection( &value->members_[index], library );
}

MessageMemberIntrospection MessageMembersIntrospection::getMember( const std::string &name ) const
{
  for ( uint32_t i = 0; i < value->member_count_; ++i ) {
    if ( value->members_[i].name_ == name )
      return MessageMemberIntrospection( &value->members_[i], library );
  }
  return {};
}

bool MessageMembersIntrospection::containsMember( const std::string &name ) const
{
  for ( uint32_t i = 0; i < value->member_count_; ++i ) {
    if ( value->members_[i].name_ == name )
      return true;
  }
  return false;
}

std::vector<std::string> MessageMembersIntrospection::memberNames() const
{
  std::vector<std::string> result;
  result.reserve( value->member_count_ );
  for ( uint32_t i = 0; i < value->member_count_; ++i ) {
    result.emplace_back( value->members_[i].name_ );
  }
  return result;
}

std::string MessageMembersIntrospection::getMessageName() const
{
  static const std::regex namespace_regex( "::" );
  return std::regex_replace( value->message_namespace_, namespace_regex, "/" ) + "/" +
         value->message_name_;
}

std::string MessageMembersIntrospection::getMessageDatatype() const
{
  return std::string( value->message_namespace_ ) + "::" + value->message_name_;
}
} // namespace ros_babel_fish
