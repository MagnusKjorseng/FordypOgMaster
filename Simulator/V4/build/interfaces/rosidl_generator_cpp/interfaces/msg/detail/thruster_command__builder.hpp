// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/ThrusterCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/thruster_command.hpp"


#ifndef INTERFACES__MSG__DETAIL__THRUSTER_COMMAND__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__THRUSTER_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/thruster_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_ThrusterCommand_angle
{
public:
  explicit Init_ThrusterCommand_angle(::interfaces::msg::ThrusterCommand & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::ThrusterCommand angle(::interfaces::msg::ThrusterCommand::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::ThrusterCommand msg_;
};

class Init_ThrusterCommand_rpm
{
public:
  Init_ThrusterCommand_rpm()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ThrusterCommand_angle rpm(::interfaces::msg::ThrusterCommand::_rpm_type arg)
  {
    msg_.rpm = std::move(arg);
    return Init_ThrusterCommand_angle(msg_);
  }

private:
  ::interfaces::msg::ThrusterCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::ThrusterCommand>()
{
  return interfaces::msg::builder::Init_ThrusterCommand_rpm();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__THRUSTER_COMMAND__BUILDER_HPP_
