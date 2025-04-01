// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/ThrusterCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/thruster_command.hpp"


#ifndef INTERFACES__MSG__DETAIL__THRUSTER_COMMAND__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__THRUSTER_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__ThrusterCommand __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__ThrusterCommand __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ThrusterCommand_
{
  using Type = ThrusterCommand_<ContainerAllocator>;

  explicit ThrusterCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rpm = 0.0f;
      this->angle = 0.0f;
    }
  }

  explicit ThrusterCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rpm = 0.0f;
      this->angle = 0.0f;
    }
  }

  // field types and members
  using _rpm_type =
    float;
  _rpm_type rpm;
  using _angle_type =
    float;
  _angle_type angle;

  // setters for named parameter idiom
  Type & set__rpm(
    const float & _arg)
  {
    this->rpm = _arg;
    return *this;
  }
  Type & set__angle(
    const float & _arg)
  {
    this->angle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::ThrusterCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::ThrusterCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::ThrusterCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::ThrusterCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::ThrusterCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::ThrusterCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::ThrusterCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::ThrusterCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::ThrusterCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::ThrusterCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__ThrusterCommand
    std::shared_ptr<interfaces::msg::ThrusterCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__ThrusterCommand
    std::shared_ptr<interfaces::msg::ThrusterCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ThrusterCommand_ & other) const
  {
    if (this->rpm != other.rpm) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    return true;
  }
  bool operator!=(const ThrusterCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ThrusterCommand_

// alias to use template instance with default allocator
using ThrusterCommand =
  interfaces::msg::ThrusterCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__THRUSTER_COMMAND__STRUCT_HPP_
