// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nanosaur_msgs:msg/Eyes.idl
// generated code does not contain a copyright notice

#ifndef NANOSAUR_MSGS__MSG__DETAIL__EYES__STRUCT_H_
#define NANOSAUR_MSGS__MSG__DETAIL__EYES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'DEFAULT'.
/**
  * Type eyes
 */
enum
{
  nanosaur_msgs__msg__Eyes__DEFAULT = 0
};

/// Constant 'SQUARE'.
enum
{
  nanosaur_msgs__msg__Eyes__SQUARE = 1
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/Eyes in the package nanosaur_msgs.
/**
  * Message header
 */
typedef struct nanosaur_msgs__msg__Eyes
{
  std_msgs__msg__Header header;
  int8_t type;
  /// eyes position
  geometry_msgs__msg__Point position;
} nanosaur_msgs__msg__Eyes;

// Struct for a sequence of nanosaur_msgs__msg__Eyes.
typedef struct nanosaur_msgs__msg__Eyes__Sequence
{
  nanosaur_msgs__msg__Eyes * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nanosaur_msgs__msg__Eyes__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NANOSAUR_MSGS__MSG__DETAIL__EYES__STRUCT_H_
