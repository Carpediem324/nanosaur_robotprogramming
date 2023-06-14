// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nanosaur_msgs:srv/EyeMessage.idl
// generated code does not contain a copyright notice

#ifndef NANOSAUR_MSGS__SRV__DETAIL__EYE_MESSAGE__STRUCT_H_
#define NANOSAUR_MSGS__SRV__DETAIL__EYE_MESSAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'BOTH'.
/**
  * Type message
  * Left and right message (All lines splitted in both display)
 */
enum
{
  nanosaur_msgs__srv__EyeMessage_Request__BOTH = 0
};

/// Constant 'LEFT'.
/**
  * Left screen
 */
enum
{
  nanosaur_msgs__srv__EyeMessage_Request__LEFT = 1
};

/// Constant 'RIGHT'.
/**
  * Right screen
 */
enum
{
  nanosaur_msgs__srv__EyeMessage_Request__RIGHT = 2
};

/// Constant 'FULL'.
/**
  * Type message
  * all messages line by line (5 lines for display)
 */
enum
{
  nanosaur_msgs__srv__EyeMessage_Request__FULL = 0
};

/// Constant 'WIDE'.
/**
  * one string screen (Will be read only the first two lines)
 */
enum
{
  nanosaur_msgs__srv__EyeMessage_Request__WIDE = 1
};

/// Constant 'TWO_LINES'.
/**
  * two strings screen (Will be read only the first array)
 */
enum
{
  nanosaur_msgs__srv__EyeMessage_Request__TWO_LINES = 2
};

/// Constant 'INF'.
/**
  * Timeout show message
  * No timeout message
 */
enum
{
  nanosaur_msgs__srv__EyeMessage_Request__INF = -1ll
};

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/EyeMessage in the package nanosaur_msgs.
typedef struct nanosaur_msgs__srv__EyeMessage_Request
{
  int8_t display;
  int8_t type;
  /// Message to send
  /// 0 - 5 = left display
  /// 6 - 11 = right display
  rosidl_runtime_c__String__Sequence message;
  int64_t timeout;
} nanosaur_msgs__srv__EyeMessage_Request;

// Struct for a sequence of nanosaur_msgs__srv__EyeMessage_Request.
typedef struct nanosaur_msgs__srv__EyeMessage_Request__Sequence
{
  nanosaur_msgs__srv__EyeMessage_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nanosaur_msgs__srv__EyeMessage_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/EyeMessage in the package nanosaur_msgs.
typedef struct nanosaur_msgs__srv__EyeMessage_Response
{
  bool done;
} nanosaur_msgs__srv__EyeMessage_Response;

// Struct for a sequence of nanosaur_msgs__srv__EyeMessage_Response.
typedef struct nanosaur_msgs__srv__EyeMessage_Response__Sequence
{
  nanosaur_msgs__srv__EyeMessage_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nanosaur_msgs__srv__EyeMessage_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NANOSAUR_MSGS__SRV__DETAIL__EYE_MESSAGE__STRUCT_H_
