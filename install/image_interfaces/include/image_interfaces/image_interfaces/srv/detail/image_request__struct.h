// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from image_interfaces:srv/ImageRequest.idl
// generated code does not contain a copyright notice

#ifndef IMAGE_INTERFACES__SRV__DETAIL__IMAGE_REQUEST__STRUCT_H_
#define IMAGE_INTERFACES__SRV__DETAIL__IMAGE_REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'image'
#include "sensor_msgs/msg/detail/image__struct.h"

/// Struct defined in srv/ImageRequest in the package image_interfaces.
typedef struct image_interfaces__srv__ImageRequest_Request
{
  sensor_msgs__msg__Image image;
} image_interfaces__srv__ImageRequest_Request;

// Struct for a sequence of image_interfaces__srv__ImageRequest_Request.
typedef struct image_interfaces__srv__ImageRequest_Request__Sequence
{
  image_interfaces__srv__ImageRequest_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} image_interfaces__srv__ImageRequest_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'apple_coordinates'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in srv/ImageRequest in the package image_interfaces.
typedef struct image_interfaces__srv__ImageRequest_Response
{
  int16_t num_apples;
  geometry_msgs__msg__Point__Sequence apple_coordinates;
} image_interfaces__srv__ImageRequest_Response;

// Struct for a sequence of image_interfaces__srv__ImageRequest_Response.
typedef struct image_interfaces__srv__ImageRequest_Response__Sequence
{
  image_interfaces__srv__ImageRequest_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} image_interfaces__srv__ImageRequest_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // IMAGE_INTERFACES__SRV__DETAIL__IMAGE_REQUEST__STRUCT_H_
