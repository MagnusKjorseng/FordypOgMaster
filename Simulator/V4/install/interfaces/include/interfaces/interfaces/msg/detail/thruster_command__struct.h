// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/ThrusterCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/thruster_command.h"


#ifndef INTERFACES__MSG__DETAIL__THRUSTER_COMMAND__STRUCT_H_
#define INTERFACES__MSG__DETAIL__THRUSTER_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/ThrusterCommand in the package interfaces.
typedef struct interfaces__msg__ThrusterCommand
{
  float rpm;
  float angle;
} interfaces__msg__ThrusterCommand;

// Struct for a sequence of interfaces__msg__ThrusterCommand.
typedef struct interfaces__msg__ThrusterCommand__Sequence
{
  interfaces__msg__ThrusterCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__ThrusterCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__THRUSTER_COMMAND__STRUCT_H_
