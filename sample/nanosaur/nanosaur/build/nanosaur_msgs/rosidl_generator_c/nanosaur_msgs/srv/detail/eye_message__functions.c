// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from nanosaur_msgs:srv/EyeMessage.idl
// generated code does not contain a copyright notice
#include "nanosaur_msgs/srv/detail/eye_message__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
nanosaur_msgs__srv__EyeMessage_Request__init(nanosaur_msgs__srv__EyeMessage_Request * msg)
{
  if (!msg) {
    return false;
  }
  // display
  // type
  // message
  if (!rosidl_runtime_c__String__Sequence__init(&msg->message, 0)) {
    nanosaur_msgs__srv__EyeMessage_Request__fini(msg);
    return false;
  }
  // timeout
  return true;
}

void
nanosaur_msgs__srv__EyeMessage_Request__fini(nanosaur_msgs__srv__EyeMessage_Request * msg)
{
  if (!msg) {
    return;
  }
  // display
  // type
  // message
  rosidl_runtime_c__String__Sequence__fini(&msg->message);
  // timeout
}

bool
nanosaur_msgs__srv__EyeMessage_Request__are_equal(const nanosaur_msgs__srv__EyeMessage_Request * lhs, const nanosaur_msgs__srv__EyeMessage_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // display
  if (lhs->display != rhs->display) {
    return false;
  }
  // type
  if (lhs->type != rhs->type) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  // timeout
  if (lhs->timeout != rhs->timeout) {
    return false;
  }
  return true;
}

bool
nanosaur_msgs__srv__EyeMessage_Request__copy(
  const nanosaur_msgs__srv__EyeMessage_Request * input,
  nanosaur_msgs__srv__EyeMessage_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // display
  output->display = input->display;
  // type
  output->type = input->type;
  // message
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  // timeout
  output->timeout = input->timeout;
  return true;
}

nanosaur_msgs__srv__EyeMessage_Request *
nanosaur_msgs__srv__EyeMessage_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nanosaur_msgs__srv__EyeMessage_Request * msg = (nanosaur_msgs__srv__EyeMessage_Request *)allocator.allocate(sizeof(nanosaur_msgs__srv__EyeMessage_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nanosaur_msgs__srv__EyeMessage_Request));
  bool success = nanosaur_msgs__srv__EyeMessage_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nanosaur_msgs__srv__EyeMessage_Request__destroy(nanosaur_msgs__srv__EyeMessage_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nanosaur_msgs__srv__EyeMessage_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nanosaur_msgs__srv__EyeMessage_Request__Sequence__init(nanosaur_msgs__srv__EyeMessage_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nanosaur_msgs__srv__EyeMessage_Request * data = NULL;

  if (size) {
    data = (nanosaur_msgs__srv__EyeMessage_Request *)allocator.zero_allocate(size, sizeof(nanosaur_msgs__srv__EyeMessage_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nanosaur_msgs__srv__EyeMessage_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nanosaur_msgs__srv__EyeMessage_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
nanosaur_msgs__srv__EyeMessage_Request__Sequence__fini(nanosaur_msgs__srv__EyeMessage_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      nanosaur_msgs__srv__EyeMessage_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

nanosaur_msgs__srv__EyeMessage_Request__Sequence *
nanosaur_msgs__srv__EyeMessage_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nanosaur_msgs__srv__EyeMessage_Request__Sequence * array = (nanosaur_msgs__srv__EyeMessage_Request__Sequence *)allocator.allocate(sizeof(nanosaur_msgs__srv__EyeMessage_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nanosaur_msgs__srv__EyeMessage_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nanosaur_msgs__srv__EyeMessage_Request__Sequence__destroy(nanosaur_msgs__srv__EyeMessage_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nanosaur_msgs__srv__EyeMessage_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nanosaur_msgs__srv__EyeMessage_Request__Sequence__are_equal(const nanosaur_msgs__srv__EyeMessage_Request__Sequence * lhs, const nanosaur_msgs__srv__EyeMessage_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nanosaur_msgs__srv__EyeMessage_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nanosaur_msgs__srv__EyeMessage_Request__Sequence__copy(
  const nanosaur_msgs__srv__EyeMessage_Request__Sequence * input,
  nanosaur_msgs__srv__EyeMessage_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nanosaur_msgs__srv__EyeMessage_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nanosaur_msgs__srv__EyeMessage_Request * data =
      (nanosaur_msgs__srv__EyeMessage_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nanosaur_msgs__srv__EyeMessage_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nanosaur_msgs__srv__EyeMessage_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nanosaur_msgs__srv__EyeMessage_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
nanosaur_msgs__srv__EyeMessage_Response__init(nanosaur_msgs__srv__EyeMessage_Response * msg)
{
  if (!msg) {
    return false;
  }
  // done
  return true;
}

void
nanosaur_msgs__srv__EyeMessage_Response__fini(nanosaur_msgs__srv__EyeMessage_Response * msg)
{
  if (!msg) {
    return;
  }
  // done
}

bool
nanosaur_msgs__srv__EyeMessage_Response__are_equal(const nanosaur_msgs__srv__EyeMessage_Response * lhs, const nanosaur_msgs__srv__EyeMessage_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // done
  if (lhs->done != rhs->done) {
    return false;
  }
  return true;
}

bool
nanosaur_msgs__srv__EyeMessage_Response__copy(
  const nanosaur_msgs__srv__EyeMessage_Response * input,
  nanosaur_msgs__srv__EyeMessage_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // done
  output->done = input->done;
  return true;
}

nanosaur_msgs__srv__EyeMessage_Response *
nanosaur_msgs__srv__EyeMessage_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nanosaur_msgs__srv__EyeMessage_Response * msg = (nanosaur_msgs__srv__EyeMessage_Response *)allocator.allocate(sizeof(nanosaur_msgs__srv__EyeMessage_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nanosaur_msgs__srv__EyeMessage_Response));
  bool success = nanosaur_msgs__srv__EyeMessage_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nanosaur_msgs__srv__EyeMessage_Response__destroy(nanosaur_msgs__srv__EyeMessage_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nanosaur_msgs__srv__EyeMessage_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nanosaur_msgs__srv__EyeMessage_Response__Sequence__init(nanosaur_msgs__srv__EyeMessage_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nanosaur_msgs__srv__EyeMessage_Response * data = NULL;

  if (size) {
    data = (nanosaur_msgs__srv__EyeMessage_Response *)allocator.zero_allocate(size, sizeof(nanosaur_msgs__srv__EyeMessage_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nanosaur_msgs__srv__EyeMessage_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nanosaur_msgs__srv__EyeMessage_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
nanosaur_msgs__srv__EyeMessage_Response__Sequence__fini(nanosaur_msgs__srv__EyeMessage_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      nanosaur_msgs__srv__EyeMessage_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

nanosaur_msgs__srv__EyeMessage_Response__Sequence *
nanosaur_msgs__srv__EyeMessage_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nanosaur_msgs__srv__EyeMessage_Response__Sequence * array = (nanosaur_msgs__srv__EyeMessage_Response__Sequence *)allocator.allocate(sizeof(nanosaur_msgs__srv__EyeMessage_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nanosaur_msgs__srv__EyeMessage_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nanosaur_msgs__srv__EyeMessage_Response__Sequence__destroy(nanosaur_msgs__srv__EyeMessage_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nanosaur_msgs__srv__EyeMessage_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nanosaur_msgs__srv__EyeMessage_Response__Sequence__are_equal(const nanosaur_msgs__srv__EyeMessage_Response__Sequence * lhs, const nanosaur_msgs__srv__EyeMessage_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nanosaur_msgs__srv__EyeMessage_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nanosaur_msgs__srv__EyeMessage_Response__Sequence__copy(
  const nanosaur_msgs__srv__EyeMessage_Response__Sequence * input,
  nanosaur_msgs__srv__EyeMessage_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nanosaur_msgs__srv__EyeMessage_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nanosaur_msgs__srv__EyeMessage_Response * data =
      (nanosaur_msgs__srv__EyeMessage_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nanosaur_msgs__srv__EyeMessage_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nanosaur_msgs__srv__EyeMessage_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nanosaur_msgs__srv__EyeMessage_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
