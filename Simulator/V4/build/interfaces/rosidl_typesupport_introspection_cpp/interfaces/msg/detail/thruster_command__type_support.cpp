// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from interfaces:msg/ThrusterCommand.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "interfaces/msg/detail/thruster_command__functions.h"
#include "interfaces/msg/detail/thruster_command__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ThrusterCommand_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) interfaces::msg::ThrusterCommand(_init);
}

void ThrusterCommand_fini_function(void * message_memory)
{
  auto typed_message = static_cast<interfaces::msg::ThrusterCommand *>(message_memory);
  typed_message->~ThrusterCommand();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ThrusterCommand_message_member_array[2] = {
  {
    "rpm",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces::msg::ThrusterCommand, rpm),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "angle",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces::msg::ThrusterCommand, angle),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ThrusterCommand_message_members = {
  "interfaces::msg",  // message namespace
  "ThrusterCommand",  // message name
  2,  // number of fields
  sizeof(interfaces::msg::ThrusterCommand),
  false,  // has_any_key_member_
  ThrusterCommand_message_member_array,  // message members
  ThrusterCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  ThrusterCommand_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ThrusterCommand_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ThrusterCommand_message_members,
  get_message_typesupport_handle_function,
  &interfaces__msg__ThrusterCommand__get_type_hash,
  &interfaces__msg__ThrusterCommand__get_type_description,
  &interfaces__msg__ThrusterCommand__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<interfaces::msg::ThrusterCommand>()
{
  return &::interfaces::msg::rosidl_typesupport_introspection_cpp::ThrusterCommand_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, interfaces, msg, ThrusterCommand)() {
  return &::interfaces::msg::rosidl_typesupport_introspection_cpp::ThrusterCommand_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
