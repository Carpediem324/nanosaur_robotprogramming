// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from nanosaur_msgs:msg/Eyes.idl
// generated code does not contain a copyright notice
#include "nanosaur_msgs/msg/detail/eyes__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
nanosaur_msgs__msg__Eyes__init(nanosaur_msgs__msg__Eyes * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    nanosaur_msgs__msg__Eyes__fini(msg);
    return false;
  }
  // type
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    nanosaur_msgs__msg__Eyes__fini(msg);
    return false;
  }
  return true;
}

void
nanosaur_msgs__msg__Eyes__fini(nanosaur_msgs__msg__Eyes * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // type
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
}

bool
nanosaur_msgs__msg__Eyes__are_equal(const nanosaur_msgs__msg__Eyes * lhs, const nanosaur_msgs__msg__Eyes * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // type
  if (lhs->type != rhs->type) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  return true;
}

bool
nanosaur_msgs__msg__Eyes__copy(
  const nanosaur_msgs__msg__Eyes * input,
  nanosaur_msgs__msg__Eyes * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // type
  output->type = input->type;
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  return true;
}

nanosaur_msgs__msg__Eyes *
nanosaur_msgs__msg__Eyes__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nanosaur_msgs__msg__Eyes * msg = (nanosaur_msgs__msg__Eyes *)allocator.allocate(sizeof(nanosaur_msgs__msg__Eyes), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nanosaur_msgs__msg__Eyes));
  bool success = nanosaur_msgs__msg__Eyes__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nanosaur_msgs__msg__Eyes__destroy(nanosaur_msgs__msg__Eyes * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nanosaur_msgs__msg__Eyes__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nanosaur_msgs__msg__Eyes__Sequence__init(nanosaur_msgs__msg__Eyes__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nanosaur_msgs__msg__Eyes * data = NULL;

  if (size) {
    data = (nanosaur_msgs__msg__Eyes *)allocator.zero_allocate(size, sizeof(nanosaur_msgs__msg__Eyes), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nanosaur_msgs__msg__Eyes__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nanosaur_msgs__msg__Eyes__fini(&data[i - 1]);
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
nanosaur_msgs__msg__Eyes__Sequence__fini(nanosaur_msgs__msg__Eyes__Sequence * array)
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
      nanosaur_msgs__msg__Eyes__fini(&array->data[i]);
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

nanosaur_msgs__msg__Eyes__Sequence *
nanosaur_msgs__msg__Eyes__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nanosaur_msgs__msg__Eyes__Sequence * array = (nanosaur_msgs__msg__Eyes__Sequence *)allocator.allocate(sizeof(nanosaur_msgs__msg__Eyes__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nanosaur_msgs__msg__Eyes__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nanosaur_msgs__msg__Eyes__Sequence__destroy(nanosaur_msgs__msg__Eyes__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nanosaur_msgs__msg__Eyes__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nanosaur_msgs__msg__Eyes__Sequence__are_equal(const nanosaur_msgs__msg__Eyes__Sequence * lhs, const nanosaur_msgs__msg__Eyes__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nanosaur_msgs__msg__Eyes__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nanosaur_msgs__msg__Eyes__Sequence__copy(
  const nanosaur_msgs__msg__Eyes__Sequence * input,
  nanosaur_msgs__msg__Eyes__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nanosaur_msgs__msg__Eyes);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nanosaur_msgs__msg__Eyes * data =
      (nanosaur_msgs__msg__Eyes *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nanosaur_msgs__msg__Eyes__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nanosaur_msgs__msg__Eyes__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nanosaur_msgs__msg__Eyes__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
