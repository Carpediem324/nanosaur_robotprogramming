// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from nanosaur_msgs:srv/EyeMessage.idl
// generated code does not contain a copyright notice

#ifndef NANOSAUR_MSGS__SRV__DETAIL__EYE_MESSAGE__TRAITS_HPP_
#define NANOSAUR_MSGS__SRV__DETAIL__EYE_MESSAGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "nanosaur_msgs/srv/detail/eye_message__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace nanosaur_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const EyeMessage_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: display
  {
    out << "display: ";
    rosidl_generator_traits::value_to_yaml(msg.display, out);
    out << ", ";
  }

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: message
  {
    if (msg.message.size() == 0) {
      out << "message: []";
    } else {
      out << "message: [";
      size_t pending_items = msg.message.size();
      for (auto item : msg.message) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: timeout
  {
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EyeMessage_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: display
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "display: ";
    rosidl_generator_traits::value_to_yaml(msg.display, out);
    out << "\n";
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

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.message.size() == 0) {
      out << "message: []\n";
    } else {
      out << "message:\n";
      for (auto item : msg.message) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EyeMessage_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace nanosaur_msgs

namespace rosidl_generator_traits
{

[[deprecated("use nanosaur_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nanosaur_msgs::srv::EyeMessage_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  nanosaur_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nanosaur_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const nanosaur_msgs::srv::EyeMessage_Request & msg)
{
  return nanosaur_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<nanosaur_msgs::srv::EyeMessage_Request>()
{
  return "nanosaur_msgs::srv::EyeMessage_Request";
}

template<>
inline const char * name<nanosaur_msgs::srv::EyeMessage_Request>()
{
  return "nanosaur_msgs/srv/EyeMessage_Request";
}

template<>
struct has_fixed_size<nanosaur_msgs::srv::EyeMessage_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<nanosaur_msgs::srv::EyeMessage_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<nanosaur_msgs::srv::EyeMessage_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace nanosaur_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const EyeMessage_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: done
  {
    out << "done: ";
    rosidl_generator_traits::value_to_yaml(msg.done, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EyeMessage_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: done
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "done: ";
    rosidl_generator_traits::value_to_yaml(msg.done, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EyeMessage_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace nanosaur_msgs

namespace rosidl_generator_traits
{

[[deprecated("use nanosaur_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nanosaur_msgs::srv::EyeMessage_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  nanosaur_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nanosaur_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const nanosaur_msgs::srv::EyeMessage_Response & msg)
{
  return nanosaur_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<nanosaur_msgs::srv::EyeMessage_Response>()
{
  return "nanosaur_msgs::srv::EyeMessage_Response";
}

template<>
inline const char * name<nanosaur_msgs::srv::EyeMessage_Response>()
{
  return "nanosaur_msgs/srv/EyeMessage_Response";
}

template<>
struct has_fixed_size<nanosaur_msgs::srv::EyeMessage_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<nanosaur_msgs::srv::EyeMessage_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<nanosaur_msgs::srv::EyeMessage_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<nanosaur_msgs::srv::EyeMessage>()
{
  return "nanosaur_msgs::srv::EyeMessage";
}

template<>
inline const char * name<nanosaur_msgs::srv::EyeMessage>()
{
  return "nanosaur_msgs/srv/EyeMessage";
}

template<>
struct has_fixed_size<nanosaur_msgs::srv::EyeMessage>
  : std::integral_constant<
    bool,
    has_fixed_size<nanosaur_msgs::srv::EyeMessage_Request>::value &&
    has_fixed_size<nanosaur_msgs::srv::EyeMessage_Response>::value
  >
{
};

template<>
struct has_bounded_size<nanosaur_msgs::srv::EyeMessage>
  : std::integral_constant<
    bool,
    has_bounded_size<nanosaur_msgs::srv::EyeMessage_Request>::value &&
    has_bounded_size<nanosaur_msgs::srv::EyeMessage_Response>::value
  >
{
};

template<>
struct is_service<nanosaur_msgs::srv::EyeMessage>
  : std::true_type
{
};

template<>
struct is_service_request<nanosaur_msgs::srv::EyeMessage_Request>
  : std::true_type
{
};

template<>
struct is_service_response<nanosaur_msgs::srv::EyeMessage_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // NANOSAUR_MSGS__SRV__DETAIL__EYE_MESSAGE__TRAITS_HPP_
