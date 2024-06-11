// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from image_interfaces:srv/ImageRequest.idl
// generated code does not contain a copyright notice

#ifndef IMAGE_INTERFACES__SRV__DETAIL__IMAGE_REQUEST__STRUCT_HPP_
#define IMAGE_INTERFACES__SRV__DETAIL__IMAGE_REQUEST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'image'
#include "sensor_msgs/msg/detail/image__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__image_interfaces__srv__ImageRequest_Request __attribute__((deprecated))
#else
# define DEPRECATED__image_interfaces__srv__ImageRequest_Request __declspec(deprecated)
#endif

namespace image_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ImageRequest_Request_
{
  using Type = ImageRequest_Request_<ContainerAllocator>;

  explicit ImageRequest_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : image(_init)
  {
    (void)_init;
  }

  explicit ImageRequest_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : image(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _image_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _image_type image;

  // setters for named parameter idiom
  Type & set__image(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->image = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    image_interfaces::srv::ImageRequest_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const image_interfaces::srv::ImageRequest_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<image_interfaces::srv::ImageRequest_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<image_interfaces::srv::ImageRequest_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      image_interfaces::srv::ImageRequest_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<image_interfaces::srv::ImageRequest_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      image_interfaces::srv::ImageRequest_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<image_interfaces::srv::ImageRequest_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<image_interfaces::srv::ImageRequest_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<image_interfaces::srv::ImageRequest_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__image_interfaces__srv__ImageRequest_Request
    std::shared_ptr<image_interfaces::srv::ImageRequest_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__image_interfaces__srv__ImageRequest_Request
    std::shared_ptr<image_interfaces::srv::ImageRequest_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ImageRequest_Request_ & other) const
  {
    if (this->image != other.image) {
      return false;
    }
    return true;
  }
  bool operator!=(const ImageRequest_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ImageRequest_Request_

// alias to use template instance with default allocator
using ImageRequest_Request =
  image_interfaces::srv::ImageRequest_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace image_interfaces


// Include directives for member types
// Member 'apple_coordinates'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__image_interfaces__srv__ImageRequest_Response __attribute__((deprecated))
#else
# define DEPRECATED__image_interfaces__srv__ImageRequest_Response __declspec(deprecated)
#endif

namespace image_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ImageRequest_Response_
{
  using Type = ImageRequest_Response_<ContainerAllocator>;

  explicit ImageRequest_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_apples = 0;
    }
  }

  explicit ImageRequest_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_apples = 0;
    }
  }

  // field types and members
  using _num_apples_type =
    int16_t;
  _num_apples_type num_apples;
  using _apple_coordinates_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _apple_coordinates_type apple_coordinates;

  // setters for named parameter idiom
  Type & set__num_apples(
    const int16_t & _arg)
  {
    this->num_apples = _arg;
    return *this;
  }
  Type & set__apple_coordinates(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->apple_coordinates = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    image_interfaces::srv::ImageRequest_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const image_interfaces::srv::ImageRequest_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<image_interfaces::srv::ImageRequest_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<image_interfaces::srv::ImageRequest_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      image_interfaces::srv::ImageRequest_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<image_interfaces::srv::ImageRequest_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      image_interfaces::srv::ImageRequest_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<image_interfaces::srv::ImageRequest_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<image_interfaces::srv::ImageRequest_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<image_interfaces::srv::ImageRequest_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__image_interfaces__srv__ImageRequest_Response
    std::shared_ptr<image_interfaces::srv::ImageRequest_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__image_interfaces__srv__ImageRequest_Response
    std::shared_ptr<image_interfaces::srv::ImageRequest_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ImageRequest_Response_ & other) const
  {
    if (this->num_apples != other.num_apples) {
      return false;
    }
    if (this->apple_coordinates != other.apple_coordinates) {
      return false;
    }
    return true;
  }
  bool operator!=(const ImageRequest_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ImageRequest_Response_

// alias to use template instance with default allocator
using ImageRequest_Response =
  image_interfaces::srv::ImageRequest_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace image_interfaces

namespace image_interfaces
{

namespace srv
{

struct ImageRequest
{
  using Request = image_interfaces::srv::ImageRequest_Request;
  using Response = image_interfaces::srv::ImageRequest_Response;
};

}  // namespace srv

}  // namespace image_interfaces

#endif  // IMAGE_INTERFACES__SRV__DETAIL__IMAGE_REQUEST__STRUCT_HPP_
