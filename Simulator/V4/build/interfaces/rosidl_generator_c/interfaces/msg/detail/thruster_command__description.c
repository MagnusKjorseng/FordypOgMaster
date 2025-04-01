// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from interfaces:msg/ThrusterCommand.idl
// generated code does not contain a copyright notice

#include "interfaces/msg/detail/thruster_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_interfaces
const rosidl_type_hash_t *
interfaces__msg__ThrusterCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc8, 0xa3, 0x03, 0x4a, 0x2c, 0x72, 0x52, 0x8c,
      0x32, 0x56, 0xc6, 0x08, 0xc7, 0x10, 0x3e, 0xaf,
      0x99, 0xc6, 0xe6, 0x98, 0xbf, 0x71, 0x4f, 0x4b,
      0x00, 0x69, 0x53, 0x07, 0xa5, 0xc5, 0x82, 0xcb,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char interfaces__msg__ThrusterCommand__TYPE_NAME[] = "interfaces/msg/ThrusterCommand";

// Define type names, field names, and default values
static char interfaces__msg__ThrusterCommand__FIELD_NAME__rpm[] = "rpm";
static char interfaces__msg__ThrusterCommand__FIELD_NAME__angle[] = "angle";

static rosidl_runtime_c__type_description__Field interfaces__msg__ThrusterCommand__FIELDS[] = {
  {
    {interfaces__msg__ThrusterCommand__FIELD_NAME__rpm, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {interfaces__msg__ThrusterCommand__FIELD_NAME__angle, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
interfaces__msg__ThrusterCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {interfaces__msg__ThrusterCommand__TYPE_NAME, 30, 30},
      {interfaces__msg__ThrusterCommand__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 rpm\n"
  "float32 angle";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
interfaces__msg__ThrusterCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {interfaces__msg__ThrusterCommand__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 26, 26},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
interfaces__msg__ThrusterCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *interfaces__msg__ThrusterCommand__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
