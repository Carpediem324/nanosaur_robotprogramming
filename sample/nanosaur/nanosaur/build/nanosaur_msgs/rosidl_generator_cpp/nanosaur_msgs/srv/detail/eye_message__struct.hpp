// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nanosaur_msgs:srv/EyeMessage.idl
// generated code does not contain a copyright notice

#ifndef NANOSAUR_MSGS__SRV__DETAIL__EYE_MESSAGE__STRUCT_HPP_
#define NANOSAUR_MSGS__SRV__DETAIL__EYE_MESSAGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__nanosaur_msgs__srv__EyeMessage_Request __attribute__((deprecated))
#else
# define DEPRECATED__nanosaur_msgs__srv__EyeMessage_Request __declspec(deprecated)
#endif

namespace nanosaur_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct EyeMessage_Request_
{
  using Type = EyeMessage_Request_<ContainerAllocator>;

  explicit EyeMessage_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->display = 0;
      this->type = 0;
      this->timeout = 0ll;
    }
  }

  explicit EyeMessage_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->display = 0;
      this->type = 0;
      this->timeout = 0ll;
    }
  }

  // field types and members
  using _display_type =
    int8_t;
  _display_type display;
  using _type_type =
    int8_t;
  _type_type type;
  using _message_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _message_type message;
  using _timeout_type =
    int64_t;
  _timeout_type timeout;

  // setters for named parameter idiom
  Type & set__display(
    const int8_t & _arg)
  {
    this->display = _arg;
    return *this;
  }
  Type & set__type(
    const int8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__message(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__timeout(
    const int64_t & _arg)
  {
    this->timeout = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t BOTH =
    0;
  static constexpr int8_t LEFT =
    1;
  static constexpr int8_t RIGHT =
    2;
  static constexpr int8_t FULL =
    0;
  static constexpr int8_t WIDE =
    1;
  static constexpr int8_t TWO_LINES =
    2;
  static constexpr int64_t INF =
    -1;

  // pointer types
  using RawPtr =
    nanosaur_msgs::srv::EyeMessage_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const nanosaur_msgs::srv::EyeMessage_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nanosaur_msgs::srv::EyeMessage_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nanosaur_msgs::srv::EyeMessage_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nanosaur_msgs::srv::EyeMessage_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nanosaur_msgs::srv::EyeMessage_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nanosaur_msgs::srv::EyeMessage_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nanosaur_msgs::srv::EyeMessage_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nanosaur_msgs::srv::EyeMessage_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nanosaur_msgs::srv::EyeMessage_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nanosaur_msgs__srv__EyeMessage_Request
    std::shared_ptr<nanosaur_msgs::srv::EyeMessage_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nanosaur_msgs__srv__EyeMessage_Request
    std::shared_ptr<nanosaur_msgs::srv::EyeMessage_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EyeMessage_Request_ & other) const
  {
    if (this->display != other.display) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->timeout != other.timeout) {
      return false;
    }
    return true;
  }
  bool operator!=(const EyeMessage_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EyeMessage_Request_

// alias to use template instance with default allocator
using EyeMessage_Request =
  nanosaur_msgs::srv::EyeMessage_Request_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int8_t EyeMessage_Request_<ContainerAllocator>::BOTH;
template<typename ContainerAllocator>
constexpr int8_t EyeMessage_Request_<ContainerAllocator>::LEFT;
template<typename ContainerAllocator>
constexpr int8_t EyeMessage_Request_<ContainerAllocator>::RIGHT;
template<typename ContainerAllocator>
constexpr int8_t EyeMessage_Request_<ContainerAllocator>::FULL;
template<typename ContainerAllocator>
constexpr int8_t EyeMessage_Request_<ContainerAllocator>::WIDE;
template<typename ContainerAllocator>
constexpr int8_t EyeMessage_Request_<ContainerAllocator>::TWO_LINES;
template<typename ContainerAllocator>
constexpr int64_t EyeMessage_Request_<ContainerAllocator>::INF;

}  // namespace srv

}  // namespace nanosaur_msgs


#ifndef _WIN32
# define DEPRECATED__nanosaur_msgs__srv__EyeMessage_Response __attribute__((deprecated))
#else
# define DEPRECATED__nanosaur_msgs__srv__EyeMessage_Response __declspec(deprecated)
#endif

namespace nanosaur_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct EyeMessage_Response_
{
  using Type = EyeMessage_Response_<ContainerAllocator>;

  explicit EyeMessage_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->done = false;
    }
  }

  explicit EyeMessage_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->done = false;
    }
  }

  // field types and members
  using _done_type =
    bool;
  _done_type done;

  // setters for named parameter idiom
  Type & set__done(
    const bool & _arg)
  {
    this->done = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nanosaur_msgs::srv::EyeMessage_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const nanosaur_msgs::srv::EyeMessage_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nanosaur_msgs::srv::EyeMessage_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nanosaur_msgs::srv::EyeMessage_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nanosaur_msgs::srv::EyeMessage_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nanosaur_msgs::srv::EyeMessage_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nanosaur_msgs::srv::EyeMessage_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nanosaur_msgs::srv::EyeMessage_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nanosaur_msgs::srv::EyeMessage_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nanosaur_msgs::srv::EyeMessage_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nanosaur_msgs__srv__EyeMessage_Response
    std::shared_ptr<nanosaur_msgs::srv::EyeMessage_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nanosaur_msgs__srv__EyeMessage_Response
    std::shared_ptr<nanosaur_msgs::srv::EyeMessage_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EyeMessage_Response_ & other) const
  {
    if (this->done != other.done) {
      return false;
    }
    return true;
  }
  bool operator!=(const EyeMessage_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EyeMessage_Response_

// alias to use template instance with default allocator
using EyeMessage_Response =
  nanosaur_msgs::srv::EyeMessage_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace nanosaur_msgs

namespace nanosaur_msgs
{

namespace srv
{

struct EyeMessage
{
  using Request = nanosaur_msgs::srv::EyeMessage_Request;
  using Response = nanosaur_msgs::srv::EyeMessage_Response;
};

}  // namespace srv

}  // namespace nanosaur_msgs

#endif  // NANOSAUR_MSGS__SRV__DETAIL__EYE_MESSAGE__STRUCT_HPP_
