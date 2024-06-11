// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interfaces:srv/ImageRequest.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__IMAGE_REQUEST__TRAITS_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__IMAGE_REQUEST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interfaces/srv/detail/image_request__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'image'
#include "sensor_msgs/msg/detail/image__traits.hpp"

namespace custom_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ImageRequest_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: image
  {
    out << "image: ";
    to_flow_style_yaml(msg.image, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ImageRequest_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: image
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "image:\n";
    to_block_style_yaml(msg.image, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ImageRequest_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::srv::ImageRequest_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::srv::ImageRequest_Request & msg)
{
  return custom_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::srv::ImageRequest_Request>()
{
  return "custom_interfaces::srv::ImageRequest_Request";
}

template<>
inline const char * name<custom_interfaces::srv::ImageRequest_Request>()
{
  return "custom_interfaces/srv/ImageRequest_Request";
}

template<>
struct has_fixed_size<custom_interfaces::srv::ImageRequest_Request>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::Image>::value> {};

template<>
struct has_bounded_size<custom_interfaces::srv::ImageRequest_Request>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::Image>::value> {};

template<>
struct is_message<custom_interfaces::srv::ImageRequest_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'apple_coordinates'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace custom_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ImageRequest_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: num_apples
  {
    out << "num_apples: ";
    rosidl_generator_traits::value_to_yaml(msg.num_apples, out);
    out << ", ";
  }

  // member: apple_coordinates
  {
    if (msg.apple_coordinates.size() == 0) {
      out << "apple_coordinates: []";
    } else {
      out << "apple_coordinates: [";
      size_t pending_items = msg.apple_coordinates.size();
      for (auto item : msg.apple_coordinates) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ImageRequest_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: num_apples
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_apples: ";
    rosidl_generator_traits::value_to_yaml(msg.num_apples, out);
    out << "\n";
  }

  // member: apple_coordinates
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.apple_coordinates.size() == 0) {
      out << "apple_coordinates: []\n";
    } else {
      out << "apple_coordinates:\n";
      for (auto item : msg.apple_coordinates) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ImageRequest_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::srv::ImageRequest_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::srv::ImageRequest_Response & msg)
{
  return custom_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::srv::ImageRequest_Response>()
{
  return "custom_interfaces::srv::ImageRequest_Response";
}

template<>
inline const char * name<custom_interfaces::srv::ImageRequest_Response>()
{
  return "custom_interfaces/srv/ImageRequest_Response";
}

template<>
struct has_fixed_size<custom_interfaces::srv::ImageRequest_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_interfaces::srv::ImageRequest_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_interfaces::srv::ImageRequest_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<custom_interfaces::srv::ImageRequest>()
{
  return "custom_interfaces::srv::ImageRequest";
}

template<>
inline const char * name<custom_interfaces::srv::ImageRequest>()
{
  return "custom_interfaces/srv/ImageRequest";
}

template<>
struct has_fixed_size<custom_interfaces::srv::ImageRequest>
  : std::integral_constant<
    bool,
    has_fixed_size<custom_interfaces::srv::ImageRequest_Request>::value &&
    has_fixed_size<custom_interfaces::srv::ImageRequest_Response>::value
  >
{
};

template<>
struct has_bounded_size<custom_interfaces::srv::ImageRequest>
  : std::integral_constant<
    bool,
    has_bounded_size<custom_interfaces::srv::ImageRequest_Request>::value &&
    has_bounded_size<custom_interfaces::srv::ImageRequest_Response>::value
  >
{
};

template<>
struct is_service<custom_interfaces::srv::ImageRequest>
  : std::true_type
{
};

template<>
struct is_service_request<custom_interfaces::srv::ImageRequest_Request>
  : std::true_type
{
};

template<>
struct is_service_response<custom_interfaces::srv::ImageRequest_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__IMAGE_REQUEST__TRAITS_HPP_
