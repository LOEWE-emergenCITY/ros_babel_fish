// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_MESSAGE_DESCRIPTION_H
#define ROS_BABEL_FISH_MESSAGE_DESCRIPTION_H

#include <rosidl_runtime_c/action_type_support_struct.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/service_type_support_struct.h>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

#include <memory>
#include <string>
#include <vector>

namespace ros_babel_fish
{

struct MessageTypeSupport {
  using SharedPtr = std::shared_ptr<MessageTypeSupport>;
  using ConstSharedPtr = std::shared_ptr<const MessageTypeSupport>;

  std::string name;

  //! In case the type_suport_handle's memory is handled elsewhere, this can be used to make sure the memory stays valid.
  std::shared_ptr<void> type_support_library;
  //! Needed to create subscribers.
  rosidl_message_type_support_t type_support_handle;

  //! Same as above.
  std::shared_ptr<void> introspection_type_support_library;
  //! Needed to parse messages. Check the message_template for null to check if this handle is valid.
  rosidl_message_type_support_t introspection_type_support_handle;
};

struct MessageMemberIntrospection {
  MessageMemberIntrospection() = default;
  MessageMemberIntrospection( const rosidl_typesupport_introspection_cpp::MessageMember *member,
                              std::shared_ptr<const void> library )
      : library( std::move( library ) ), value( member )
  {
  }

  const rosidl_typesupport_introspection_cpp::MessageMember *operator->() const { return value; }

  bool isNull() const { return value == nullptr; }

  std::shared_ptr<const void> library;
  const rosidl_typesupport_introspection_cpp::MessageMember *value = nullptr;
};

struct MessageMembersIntrospection {
  /* implicit */ MessageMembersIntrospection( const MessageTypeSupport &type_support ) // NOLINT
      : library( type_support.introspection_type_support_library ),
        value( static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
            type_support.introspection_type_support_handle.data ) )
  {
  }

  /* implicit */ MessageMembersIntrospection( const MessageMemberIntrospection &member ) // NOLINT
      : library( member.library ),
        value( static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
            member.value->members_->data ) )
  {
  }

  MessageMembersIntrospection( const rosidl_typesupport_introspection_cpp::MessageMembers *members,
                               std::shared_ptr<const void> library )
      : library( std::move( library ) ), value( members )
  {
  }

  const rosidl_typesupport_introspection_cpp::MessageMembers *operator->() const { return value; }

  MessageMemberIntrospection getMember( size_t index ) const;

  MessageMemberIntrospection getMember( const std::string &name ) const;

  bool containsMember( const std::string &name ) const;

  std::vector<std::string> memberNames() const;

  //! Get the name of the message represented by this introspection. E.g. "std_msgs/msg/Int32"
  std::string getMessageName() const;

  //! Get the type of the message represented by this introspection. E.g. "std_msgs::msg::Int32"
  std::string getMessageDatatype() const;

  std::shared_ptr<const void> library;
  const rosidl_typesupport_introspection_cpp::MessageMembers *value;
};

struct ServiceTypeSupport {
  using SharedPtr = std::shared_ptr<ServiceTypeSupport>;
  using ConstSharedPtr = std::shared_ptr<const ServiceTypeSupport>;

  std::string name;

  //! In case the type_suport_handle's memory is handled elsewhere, this can be used to make sure the memory stays valid.
  std::shared_ptr<void> type_support_library;
  //! Needed to create requests / responses and advertise services
  rosidl_service_type_support_t type_support_handle;

  //! Same as above.
  std::shared_ptr<void> introspection_type_support_library;
  //! Needed to parse messages. Check the message_template for null to check if this handle is valid.
  rosidl_service_type_support_t introspection_type_support_handle;

  MessageMembersIntrospection request() const
  {
    const auto *service = static_cast<const rosidl_typesupport_introspection_cpp::ServiceMembers *>(
        introspection_type_support_handle.data );
    return { service->request_members_, introspection_type_support_library };
  }

  MessageMembersIntrospection response() const
  {
    const auto *service = static_cast<const rosidl_typesupport_introspection_cpp::ServiceMembers *>(
        introspection_type_support_handle.data );
    return { service->response_members_, introspection_type_support_library };
  }
};

struct ActionTypeSupport {
  using SharedPtr = std::shared_ptr<ActionTypeSupport>;
  using ConstSharedPtr = std::shared_ptr<const ActionTypeSupport>;

  std::string name;

  //! In case the type_suport_handle's memory is handled elsewhere, this can be used to make sure the memory stays valid.
  std::shared_ptr<void> type_support_library;
  //! Needed to create subscribers.
  rosidl_action_type_support_t type_support_handle;

  //! Same as above.
  std::shared_ptr<void> introspection_type_support_library;
  //! Needed to parse messages. Check the message_template for null to check if this handle is valid.
  rosidl_action_type_support_t introspection_type_support_handle;

  ServiceTypeSupport::ConstSharedPtr goal_service_type_support;
  ServiceTypeSupport::ConstSharedPtr cancel_service_type_support;
  ServiceTypeSupport::ConstSharedPtr result_service_type_support;
  //! Feedback message with generic fields wrapping the feedback message.
  MessageTypeSupport::ConstSharedPtr feedback_message_type_support;
  MessageTypeSupport::ConstSharedPtr status_message_type_support;
};
} // namespace ros_babel_fish

#endif // ROS_BABEL_FISH_MESSAGE_DESCRIPTION_H
