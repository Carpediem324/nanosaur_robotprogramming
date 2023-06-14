// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from nanosaur_msgs:msg/Eyes.idl
// generated code does not contain a copyright notice

#ifndef NANOSAUR_MSGS__MSG__DETAIL__EYES__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define NANOSAUR_MSGS__MSG__DETAIL__EYES__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "nanosaur_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "nanosaur_msgs/msg/detail/eyes__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace nanosaur_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_nanosaur_msgs
cdr_serialize(
  const nanosaur_msgs::msg::Eyes & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_nanosaur_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  nanosaur_msgs::msg::Eyes & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_nanosaur_msgs
get_serialized_size(
  const nanosaur_msgs::msg::Eyes & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_nanosaur_msgs
max_serialized_size_Eyes(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace nanosaur_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_nanosaur_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, nanosaur_msgs, msg, Eyes)();

#ifdef __cplusplus
}
#endif

#endif  // NANOSAUR_MSGS__MSG__DETAIL__EYES__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
