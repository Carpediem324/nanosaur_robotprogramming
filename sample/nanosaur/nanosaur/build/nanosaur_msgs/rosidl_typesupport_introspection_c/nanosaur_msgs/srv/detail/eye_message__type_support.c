// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from nanosaur_msgs:srv/EyeMessage.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "nanosaur_msgs/srv/detail/eye_message__rosidl_typesupport_introspection_c.h"
#include "nanosaur_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "nanosaur_msgs/srv/detail/eye_message__functions.h"
#include "nanosaur_msgs/srv/detail/eye_message__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__EyeMessage_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  nanosaur_msgs__srv__EyeMessage_Request__init(message_memory);
}

void nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__EyeMessage_Request_fini_function(void * message_memory)
{
  nanosaur_msgs__srv__EyeMessage_Request__fini(message_memory);
}

size_t nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__size_function__EyeMessage_Request__message(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__get_const_function__EyeMessage_Request__message(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__get_function__EyeMessage_Request__message(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__fetch_function__EyeMessage_Request__message(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__get_const_function__EyeMessage_Request__message(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__assign_function__EyeMessage_Request__message(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__get_function__EyeMessage_Request__message(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__resize_function__EyeMessage_Request__message(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__EyeMessage_Request_message_member_array[4] = {
  {
    "display",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nanosaur_msgs__srv__EyeMessage_Request, display),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nanosaur_msgs__srv__EyeMessage_Request, type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nanosaur_msgs__srv__EyeMessage_Request, message),  // bytes offset in struct
    NULL,  // default value
    nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__size_function__EyeMessage_Request__message,  // size() function pointer
    nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__get_const_function__EyeMessage_Request__message,  // get_const(index) function pointer
    nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__get_function__EyeMessage_Request__message,  // get(index) function pointer
    nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__fetch_function__EyeMessage_Request__message,  // fetch(index, &value) function pointer
    nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__assign_function__EyeMessage_Request__message,  // assign(index, value) function pointer
    nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__resize_function__EyeMessage_Request__message  // resize(index) function pointer
  },
  {
    "timeout",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nanosaur_msgs__srv__EyeMessage_Request, timeout),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__EyeMessage_Request_message_members = {
  "nanosaur_msgs__srv",  // message namespace
  "EyeMessage_Request",  // message name
  4,  // number of fields
  sizeof(nanosaur_msgs__srv__EyeMessage_Request),
  nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__EyeMessage_Request_message_member_array,  // message members
  nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__EyeMessage_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__EyeMessage_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__EyeMessage_Request_message_type_support_handle = {
  0,
  &nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__EyeMessage_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_nanosaur_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nanosaur_msgs, srv, EyeMessage_Request)() {
  if (!nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__EyeMessage_Request_message_type_support_handle.typesupport_identifier) {
    nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__EyeMessage_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &nanosaur_msgs__srv__EyeMessage_Request__rosidl_typesupport_introspection_c__EyeMessage_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "nanosaur_msgs/srv/detail/eye_message__rosidl_typesupport_introspection_c.h"
// already included above
// #include "nanosaur_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "nanosaur_msgs/srv/detail/eye_message__functions.h"
// already included above
// #include "nanosaur_msgs/srv/detail/eye_message__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void nanosaur_msgs__srv__EyeMessage_Response__rosidl_typesupport_introspection_c__EyeMessage_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  nanosaur_msgs__srv__EyeMessage_Response__init(message_memory);
}

void nanosaur_msgs__srv__EyeMessage_Response__rosidl_typesupport_introspection_c__EyeMessage_Response_fini_function(void * message_memory)
{
  nanosaur_msgs__srv__EyeMessage_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember nanosaur_msgs__srv__EyeMessage_Response__rosidl_typesupport_introspection_c__EyeMessage_Response_message_member_array[1] = {
  {
    "done",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nanosaur_msgs__srv__EyeMessage_Response, done),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers nanosaur_msgs__srv__EyeMessage_Response__rosidl_typesupport_introspection_c__EyeMessage_Response_message_members = {
  "nanosaur_msgs__srv",  // message namespace
  "EyeMessage_Response",  // message name
  1,  // number of fields
  sizeof(nanosaur_msgs__srv__EyeMessage_Response),
  nanosaur_msgs__srv__EyeMessage_Response__rosidl_typesupport_introspection_c__EyeMessage_Response_message_member_array,  // message members
  nanosaur_msgs__srv__EyeMessage_Response__rosidl_typesupport_introspection_c__EyeMessage_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  nanosaur_msgs__srv__EyeMessage_Response__rosidl_typesupport_introspection_c__EyeMessage_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t nanosaur_msgs__srv__EyeMessage_Response__rosidl_typesupport_introspection_c__EyeMessage_Response_message_type_support_handle = {
  0,
  &nanosaur_msgs__srv__EyeMessage_Response__rosidl_typesupport_introspection_c__EyeMessage_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_nanosaur_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nanosaur_msgs, srv, EyeMessage_Response)() {
  if (!nanosaur_msgs__srv__EyeMessage_Response__rosidl_typesupport_introspection_c__EyeMessage_Response_message_type_support_handle.typesupport_identifier) {
    nanosaur_msgs__srv__EyeMessage_Response__rosidl_typesupport_introspection_c__EyeMessage_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &nanosaur_msgs__srv__EyeMessage_Response__rosidl_typesupport_introspection_c__EyeMessage_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "nanosaur_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "nanosaur_msgs/srv/detail/eye_message__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers nanosaur_msgs__srv__detail__eye_message__rosidl_typesupport_introspection_c__EyeMessage_service_members = {
  "nanosaur_msgs__srv",  // service namespace
  "EyeMessage",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // nanosaur_msgs__srv__detail__eye_message__rosidl_typesupport_introspection_c__EyeMessage_Request_message_type_support_handle,
  NULL  // response message
  // nanosaur_msgs__srv__detail__eye_message__rosidl_typesupport_introspection_c__EyeMessage_Response_message_type_support_handle
};

static rosidl_service_type_support_t nanosaur_msgs__srv__detail__eye_message__rosidl_typesupport_introspection_c__EyeMessage_service_type_support_handle = {
  0,
  &nanosaur_msgs__srv__detail__eye_message__rosidl_typesupport_introspection_c__EyeMessage_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nanosaur_msgs, srv, EyeMessage_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nanosaur_msgs, srv, EyeMessage_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_nanosaur_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nanosaur_msgs, srv, EyeMessage)() {
  if (!nanosaur_msgs__srv__detail__eye_message__rosidl_typesupport_introspection_c__EyeMessage_service_type_support_handle.typesupport_identifier) {
    nanosaur_msgs__srv__detail__eye_message__rosidl_typesupport_introspection_c__EyeMessage_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)nanosaur_msgs__srv__detail__eye_message__rosidl_typesupport_introspection_c__EyeMessage_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nanosaur_msgs, srv, EyeMessage_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nanosaur_msgs, srv, EyeMessage_Response)()->data;
  }

  return &nanosaur_msgs__srv__detail__eye_message__rosidl_typesupport_introspection_c__EyeMessage_service_type_support_handle;
}
