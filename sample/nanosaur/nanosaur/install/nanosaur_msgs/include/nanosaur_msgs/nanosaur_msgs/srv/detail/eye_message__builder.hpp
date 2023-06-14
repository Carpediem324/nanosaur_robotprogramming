// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nanosaur_msgs:srv/EyeMessage.idl
// generated code does not contain a copyright notice

#ifndef NANOSAUR_MSGS__SRV__DETAIL__EYE_MESSAGE__BUILDER_HPP_
#define NANOSAUR_MSGS__SRV__DETAIL__EYE_MESSAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "nanosaur_msgs/srv/detail/eye_message__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace nanosaur_msgs
{

namespace srv
{

namespace builder
{

class Init_EyeMessage_Request_timeout
{
public:
  explicit Init_EyeMessage_Request_timeout(::nanosaur_msgs::srv::EyeMessage_Request & msg)
  : msg_(msg)
  {}
  ::nanosaur_msgs::srv::EyeMessage_Request timeout(::nanosaur_msgs::srv::EyeMessage_Request::_timeout_type arg)
  {
    msg_.timeout = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nanosaur_msgs::srv::EyeMessage_Request msg_;
};

class Init_EyeMessage_Request_message
{
public:
  explicit Init_EyeMessage_Request_message(::nanosaur_msgs::srv::EyeMessage_Request & msg)
  : msg_(msg)
  {}
  Init_EyeMessage_Request_timeout message(::nanosaur_msgs::srv::EyeMessage_Request::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_EyeMessage_Request_timeout(msg_);
  }

private:
  ::nanosaur_msgs::srv::EyeMessage_Request msg_;
};

class Init_EyeMessage_Request_type
{
public:
  explicit Init_EyeMessage_Request_type(::nanosaur_msgs::srv::EyeMessage_Request & msg)
  : msg_(msg)
  {}
  Init_EyeMessage_Request_message type(::nanosaur_msgs::srv::EyeMessage_Request::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_EyeMessage_Request_message(msg_);
  }

private:
  ::nanosaur_msgs::srv::EyeMessage_Request msg_;
};

class Init_EyeMessage_Request_display
{
public:
  Init_EyeMessage_Request_display()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EyeMessage_Request_type display(::nanosaur_msgs::srv::EyeMessage_Request::_display_type arg)
  {
    msg_.display = std::move(arg);
    return Init_EyeMessage_Request_type(msg_);
  }

private:
  ::nanosaur_msgs::srv::EyeMessage_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::nanosaur_msgs::srv::EyeMessage_Request>()
{
  return nanosaur_msgs::srv::builder::Init_EyeMessage_Request_display();
}

}  // namespace nanosaur_msgs


namespace nanosaur_msgs
{

namespace srv
{

namespace builder
{

class Init_EyeMessage_Response_done
{
public:
  Init_EyeMessage_Response_done()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::nanosaur_msgs::srv::EyeMessage_Response done(::nanosaur_msgs::srv::EyeMessage_Response::_done_type arg)
  {
    msg_.done = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nanosaur_msgs::srv::EyeMessage_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::nanosaur_msgs::srv::EyeMessage_Response>()
{
  return nanosaur_msgs::srv::builder::Init_EyeMessage_Response_done();
}

}  // namespace nanosaur_msgs

#endif  // NANOSAUR_MSGS__SRV__DETAIL__EYE_MESSAGE__BUILDER_HPP_
