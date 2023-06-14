// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nanosaur_msgs:msg/Eyes.idl
// generated code does not contain a copyright notice

#ifndef NANOSAUR_MSGS__MSG__DETAIL__EYES__BUILDER_HPP_
#define NANOSAUR_MSGS__MSG__DETAIL__EYES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "nanosaur_msgs/msg/detail/eyes__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace nanosaur_msgs
{

namespace msg
{

namespace builder
{

class Init_Eyes_position
{
public:
  explicit Init_Eyes_position(::nanosaur_msgs::msg::Eyes & msg)
  : msg_(msg)
  {}
  ::nanosaur_msgs::msg::Eyes position(::nanosaur_msgs::msg::Eyes::_position_type arg)
  {
    msg_.position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nanosaur_msgs::msg::Eyes msg_;
};

class Init_Eyes_type
{
public:
  explicit Init_Eyes_type(::nanosaur_msgs::msg::Eyes & msg)
  : msg_(msg)
  {}
  Init_Eyes_position type(::nanosaur_msgs::msg::Eyes::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_Eyes_position(msg_);
  }

private:
  ::nanosaur_msgs::msg::Eyes msg_;
};

class Init_Eyes_header
{
public:
  Init_Eyes_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Eyes_type header(::nanosaur_msgs::msg::Eyes::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Eyes_type(msg_);
  }

private:
  ::nanosaur_msgs::msg::Eyes msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::nanosaur_msgs::msg::Eyes>()
{
  return nanosaur_msgs::msg::builder::Init_Eyes_header();
}

}  // namespace nanosaur_msgs

#endif  // NANOSAUR_MSGS__MSG__DETAIL__EYES__BUILDER_HPP_
