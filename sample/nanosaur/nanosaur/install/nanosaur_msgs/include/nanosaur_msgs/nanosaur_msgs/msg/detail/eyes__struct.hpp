// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nanosaur_msgs:msg/Eyes.idl
// generated code does not contain a copyright notice

#ifndef NANOSAUR_MSGS__MSG__DETAIL__EYES__STRUCT_HPP_
#define NANOSAUR_MSGS__MSG__DETAIL__EYES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__nanosaur_msgs__msg__Eyes __attribute__((deprecated))
#else
# define DEPRECATED__nanosaur_msgs__msg__Eyes __declspec(deprecated)
#endif

namespace nanosaur_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Eyes_
{
  using Type = Eyes_<ContainerAllocator>;

  explicit Eyes_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
    }
  }

  explicit Eyes_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _type_type =
    int8_t;
  _type_type type;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__type(
    const int8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t DEFAULT =
    0;
  static constexpr int8_t SQUARE =
    1;

  // pointer types
  using RawPtr =
    nanosaur_msgs::msg::Eyes_<ContainerAllocator> *;
  using ConstRawPtr =
    const nanosaur_msgs::msg::Eyes_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nanosaur_msgs::msg::Eyes_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nanosaur_msgs::msg::Eyes_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nanosaur_msgs::msg::Eyes_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nanosaur_msgs::msg::Eyes_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nanosaur_msgs::msg::Eyes_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nanosaur_msgs::msg::Eyes_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nanosaur_msgs::msg::Eyes_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nanosaur_msgs::msg::Eyes_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nanosaur_msgs__msg__Eyes
    std::shared_ptr<nanosaur_msgs::msg::Eyes_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nanosaur_msgs__msg__Eyes
    std::shared_ptr<nanosaur_msgs::msg::Eyes_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Eyes_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    return true;
  }
  bool operator!=(const Eyes_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Eyes_

// alias to use template instance with default allocator
using Eyes =
  nanosaur_msgs::msg::Eyes_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int8_t Eyes_<ContainerAllocator>::DEFAULT;
template<typename ContainerAllocator>
constexpr int8_t Eyes_<ContainerAllocator>::SQUARE;

}  // namespace msg

}  // namespace nanosaur_msgs

#endif  // NANOSAUR_MSGS__MSG__DETAIL__EYES__STRUCT_HPP_
