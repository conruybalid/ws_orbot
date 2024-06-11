// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from image_interfaces:srv/ImageRequest.idl
// generated code does not contain a copyright notice

#ifndef IMAGE_INTERFACES__SRV__DETAIL__IMAGE_REQUEST__BUILDER_HPP_
#define IMAGE_INTERFACES__SRV__DETAIL__IMAGE_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "image_interfaces/srv/detail/image_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace image_interfaces
{

namespace srv
{

namespace builder
{

class Init_ImageRequest_Request_image
{
public:
  Init_ImageRequest_Request_image()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::image_interfaces::srv::ImageRequest_Request image(::image_interfaces::srv::ImageRequest_Request::_image_type arg)
  {
    msg_.image = std::move(arg);
    return std::move(msg_);
  }

private:
  ::image_interfaces::srv::ImageRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::image_interfaces::srv::ImageRequest_Request>()
{
  return image_interfaces::srv::builder::Init_ImageRequest_Request_image();
}

}  // namespace image_interfaces


namespace image_interfaces
{

namespace srv
{

namespace builder
{

class Init_ImageRequest_Response_apple_coordinates
{
public:
  explicit Init_ImageRequest_Response_apple_coordinates(::image_interfaces::srv::ImageRequest_Response & msg)
  : msg_(msg)
  {}
  ::image_interfaces::srv::ImageRequest_Response apple_coordinates(::image_interfaces::srv::ImageRequest_Response::_apple_coordinates_type arg)
  {
    msg_.apple_coordinates = std::move(arg);
    return std::move(msg_);
  }

private:
  ::image_interfaces::srv::ImageRequest_Response msg_;
};

class Init_ImageRequest_Response_num_apples
{
public:
  Init_ImageRequest_Response_num_apples()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ImageRequest_Response_apple_coordinates num_apples(::image_interfaces::srv::ImageRequest_Response::_num_apples_type arg)
  {
    msg_.num_apples = std::move(arg);
    return Init_ImageRequest_Response_apple_coordinates(msg_);
  }

private:
  ::image_interfaces::srv::ImageRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::image_interfaces::srv::ImageRequest_Response>()
{
  return image_interfaces::srv::builder::Init_ImageRequest_Response_num_apples();
}

}  // namespace image_interfaces

#endif  // IMAGE_INTERFACES__SRV__DETAIL__IMAGE_REQUEST__BUILDER_HPP_
