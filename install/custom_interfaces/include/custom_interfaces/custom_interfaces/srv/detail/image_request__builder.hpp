// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:srv/ImageRequest.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__IMAGE_REQUEST__BUILDER_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__IMAGE_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/srv/detail/image_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
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
  ::custom_interfaces::srv::ImageRequest_Request image(::custom_interfaces::srv::ImageRequest_Request::_image_type arg)
  {
    msg_.image = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::ImageRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::ImageRequest_Request>()
{
  return custom_interfaces::srv::builder::Init_ImageRequest_Request_image();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_ImageRequest_Response_apple_coordinates
{
public:
  explicit Init_ImageRequest_Response_apple_coordinates(::custom_interfaces::srv::ImageRequest_Response & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::srv::ImageRequest_Response apple_coordinates(::custom_interfaces::srv::ImageRequest_Response::_apple_coordinates_type arg)
  {
    msg_.apple_coordinates = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::ImageRequest_Response msg_;
};

class Init_ImageRequest_Response_num_apples
{
public:
  Init_ImageRequest_Response_num_apples()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ImageRequest_Response_apple_coordinates num_apples(::custom_interfaces::srv::ImageRequest_Response::_num_apples_type arg)
  {
    msg_.num_apples = std::move(arg);
    return Init_ImageRequest_Response_apple_coordinates(msg_);
  }

private:
  ::custom_interfaces::srv::ImageRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::ImageRequest_Response>()
{
  return custom_interfaces::srv::builder::Init_ImageRequest_Response_num_apples();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__IMAGE_REQUEST__BUILDER_HPP_
