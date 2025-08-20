// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from blueboat_interfaces:srv/SetTarget.idl
// generated code does not contain a copyright notice
#include "blueboat_interfaces/srv/detail/set_target__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
blueboat_interfaces__srv__SetTarget_Request__init(blueboat_interfaces__srv__SetTarget_Request * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  return true;
}

void
blueboat_interfaces__srv__SetTarget_Request__fini(blueboat_interfaces__srv__SetTarget_Request * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
}

bool
blueboat_interfaces__srv__SetTarget_Request__are_equal(const blueboat_interfaces__srv__SetTarget_Request * lhs, const blueboat_interfaces__srv__SetTarget_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  return true;
}

bool
blueboat_interfaces__srv__SetTarget_Request__copy(
  const blueboat_interfaces__srv__SetTarget_Request * input,
  blueboat_interfaces__srv__SetTarget_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  return true;
}

blueboat_interfaces__srv__SetTarget_Request *
blueboat_interfaces__srv__SetTarget_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  blueboat_interfaces__srv__SetTarget_Request * msg = (blueboat_interfaces__srv__SetTarget_Request *)allocator.allocate(sizeof(blueboat_interfaces__srv__SetTarget_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(blueboat_interfaces__srv__SetTarget_Request));
  bool success = blueboat_interfaces__srv__SetTarget_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
blueboat_interfaces__srv__SetTarget_Request__destroy(blueboat_interfaces__srv__SetTarget_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    blueboat_interfaces__srv__SetTarget_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
blueboat_interfaces__srv__SetTarget_Request__Sequence__init(blueboat_interfaces__srv__SetTarget_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  blueboat_interfaces__srv__SetTarget_Request * data = NULL;

  if (size) {
    data = (blueboat_interfaces__srv__SetTarget_Request *)allocator.zero_allocate(size, sizeof(blueboat_interfaces__srv__SetTarget_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = blueboat_interfaces__srv__SetTarget_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        blueboat_interfaces__srv__SetTarget_Request__fini(&data[i - 1]);
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
blueboat_interfaces__srv__SetTarget_Request__Sequence__fini(blueboat_interfaces__srv__SetTarget_Request__Sequence * array)
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
      blueboat_interfaces__srv__SetTarget_Request__fini(&array->data[i]);
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

blueboat_interfaces__srv__SetTarget_Request__Sequence *
blueboat_interfaces__srv__SetTarget_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  blueboat_interfaces__srv__SetTarget_Request__Sequence * array = (blueboat_interfaces__srv__SetTarget_Request__Sequence *)allocator.allocate(sizeof(blueboat_interfaces__srv__SetTarget_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = blueboat_interfaces__srv__SetTarget_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
blueboat_interfaces__srv__SetTarget_Request__Sequence__destroy(blueboat_interfaces__srv__SetTarget_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    blueboat_interfaces__srv__SetTarget_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
blueboat_interfaces__srv__SetTarget_Request__Sequence__are_equal(const blueboat_interfaces__srv__SetTarget_Request__Sequence * lhs, const blueboat_interfaces__srv__SetTarget_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!blueboat_interfaces__srv__SetTarget_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
blueboat_interfaces__srv__SetTarget_Request__Sequence__copy(
  const blueboat_interfaces__srv__SetTarget_Request__Sequence * input,
  blueboat_interfaces__srv__SetTarget_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(blueboat_interfaces__srv__SetTarget_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    blueboat_interfaces__srv__SetTarget_Request * data =
      (blueboat_interfaces__srv__SetTarget_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!blueboat_interfaces__srv__SetTarget_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          blueboat_interfaces__srv__SetTarget_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!blueboat_interfaces__srv__SetTarget_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
blueboat_interfaces__srv__SetTarget_Response__init(blueboat_interfaces__srv__SetTarget_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  return true;
}

void
blueboat_interfaces__srv__SetTarget_Response__fini(blueboat_interfaces__srv__SetTarget_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
}

bool
blueboat_interfaces__srv__SetTarget_Response__are_equal(const blueboat_interfaces__srv__SetTarget_Response * lhs, const blueboat_interfaces__srv__SetTarget_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  return true;
}

bool
blueboat_interfaces__srv__SetTarget_Response__copy(
  const blueboat_interfaces__srv__SetTarget_Response * input,
  blueboat_interfaces__srv__SetTarget_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  return true;
}

blueboat_interfaces__srv__SetTarget_Response *
blueboat_interfaces__srv__SetTarget_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  blueboat_interfaces__srv__SetTarget_Response * msg = (blueboat_interfaces__srv__SetTarget_Response *)allocator.allocate(sizeof(blueboat_interfaces__srv__SetTarget_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(blueboat_interfaces__srv__SetTarget_Response));
  bool success = blueboat_interfaces__srv__SetTarget_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
blueboat_interfaces__srv__SetTarget_Response__destroy(blueboat_interfaces__srv__SetTarget_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    blueboat_interfaces__srv__SetTarget_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
blueboat_interfaces__srv__SetTarget_Response__Sequence__init(blueboat_interfaces__srv__SetTarget_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  blueboat_interfaces__srv__SetTarget_Response * data = NULL;

  if (size) {
    data = (blueboat_interfaces__srv__SetTarget_Response *)allocator.zero_allocate(size, sizeof(blueboat_interfaces__srv__SetTarget_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = blueboat_interfaces__srv__SetTarget_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        blueboat_interfaces__srv__SetTarget_Response__fini(&data[i - 1]);
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
blueboat_interfaces__srv__SetTarget_Response__Sequence__fini(blueboat_interfaces__srv__SetTarget_Response__Sequence * array)
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
      blueboat_interfaces__srv__SetTarget_Response__fini(&array->data[i]);
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

blueboat_interfaces__srv__SetTarget_Response__Sequence *
blueboat_interfaces__srv__SetTarget_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  blueboat_interfaces__srv__SetTarget_Response__Sequence * array = (blueboat_interfaces__srv__SetTarget_Response__Sequence *)allocator.allocate(sizeof(blueboat_interfaces__srv__SetTarget_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = blueboat_interfaces__srv__SetTarget_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
blueboat_interfaces__srv__SetTarget_Response__Sequence__destroy(blueboat_interfaces__srv__SetTarget_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    blueboat_interfaces__srv__SetTarget_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
blueboat_interfaces__srv__SetTarget_Response__Sequence__are_equal(const blueboat_interfaces__srv__SetTarget_Response__Sequence * lhs, const blueboat_interfaces__srv__SetTarget_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!blueboat_interfaces__srv__SetTarget_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
blueboat_interfaces__srv__SetTarget_Response__Sequence__copy(
  const blueboat_interfaces__srv__SetTarget_Response__Sequence * input,
  blueboat_interfaces__srv__SetTarget_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(blueboat_interfaces__srv__SetTarget_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    blueboat_interfaces__srv__SetTarget_Response * data =
      (blueboat_interfaces__srv__SetTarget_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!blueboat_interfaces__srv__SetTarget_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          blueboat_interfaces__srv__SetTarget_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!blueboat_interfaces__srv__SetTarget_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
