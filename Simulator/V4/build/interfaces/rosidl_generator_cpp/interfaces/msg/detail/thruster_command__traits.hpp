// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:msg/ThrusterCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/thruster_command.hpp"


#ifndef INTERFACES__MSG__DETAIL__THRUSTER_COMMAND__TRAITS_HPP_
#define INTERFACES__MSG__DETAIL__THRUSTER_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/msg/detail/thruster_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ThrusterCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: rpm
  {
    out << "rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.rpm, out);
    out << ", ";
  }

  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ThrusterCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: rpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.rpm, out);
    out << "\n";
  }

  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ThrusterCommand & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::msg::ThrusterCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::msg::ThrusterCommand & msg)
{
  return interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::msg::ThrusterCommand>()
{
  return "interfaces::msg::ThrusterCommand";
}

template<>
inline const char * name<interfaces::msg::ThrusterCommand>()
{
  return "interfaces/msg/ThrusterCommand";
}

template<>
struct has_fixed_size<interfaces::msg::ThrusterCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interfaces::msg::ThrusterCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interfaces::msg::ThrusterCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__MSG__DETAIL__THRUSTER_COMMAND__TRAITS_HPP_
