// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from nanosaur_msgs:msg/Eyes.idl
// generated code does not contain a copyright notice

#ifndef NANOSAUR_MSGS__MSG__DETAIL__EYES__TRAITS_HPP_
#define NANOSAUR_MSGS__MSG__DETAIL__EYES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "nanosaur_msgs/msg/detail/eyes__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace nanosaur_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Eyes & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Eyes & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Eyes & msg, bool use_flow_style = false)
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

}  // namespace nanosaur_msgs

namespace rosidl_generator_traits
{

[[deprecated("use nanosaur_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nanosaur_msgs::msg::Eyes & msg,
  std::ostream & out, size_t indentation = 0)
{
  nanosaur_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nanosaur_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const nanosaur_msgs::msg::Eyes & msg)
{
  return nanosaur_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<nanosaur_msgs::msg::Eyes>()
{
  return "nanosaur_msgs::msg::Eyes";
}

template<>
inline const char * name<nanosaur_msgs::msg::Eyes>()
{
  return "nanosaur_msgs/msg/Eyes";
}

template<>
struct has_fixed_size<nanosaur_msgs::msg::Eyes>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<nanosaur_msgs::msg::Eyes>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<nanosaur_msgs::msg::Eyes>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // NANOSAUR_MSGS__MSG__DETAIL__EYES__TRAITS_HPP_
