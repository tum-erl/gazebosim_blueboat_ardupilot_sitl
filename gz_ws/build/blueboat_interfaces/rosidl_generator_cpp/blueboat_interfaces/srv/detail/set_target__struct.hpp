// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from blueboat_interfaces:srv/SetTarget.idl
// generated code does not contain a copyright notice

#ifndef BLUEBOAT_INTERFACES__SRV__DETAIL__SET_TARGET__STRUCT_HPP_
#define BLUEBOAT_INTERFACES__SRV__DETAIL__SET_TARGET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__blueboat_interfaces__srv__SetTarget_Request __attribute__((deprecated))
#else
# define DEPRECATED__blueboat_interfaces__srv__SetTarget_Request __declspec(deprecated)
#endif

namespace blueboat_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetTarget_Request_
{
  using Type = SetTarget_Request_<ContainerAllocator>;

  explicit SetTarget_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
    }
  }

  explicit SetTarget_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
    }
  }

  // field types and members
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;

  // setters for named parameter idiom
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    blueboat_interfaces::srv::SetTarget_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const blueboat_interfaces::srv::SetTarget_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<blueboat_interfaces::srv::SetTarget_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<blueboat_interfaces::srv::SetTarget_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      blueboat_interfaces::srv::SetTarget_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<blueboat_interfaces::srv::SetTarget_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      blueboat_interfaces::srv::SetTarget_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<blueboat_interfaces::srv::SetTarget_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<blueboat_interfaces::srv::SetTarget_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<blueboat_interfaces::srv::SetTarget_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__blueboat_interfaces__srv__SetTarget_Request
    std::shared_ptr<blueboat_interfaces::srv::SetTarget_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__blueboat_interfaces__srv__SetTarget_Request
    std::shared_ptr<blueboat_interfaces::srv::SetTarget_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetTarget_Request_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetTarget_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetTarget_Request_

// alias to use template instance with default allocator
using SetTarget_Request =
  blueboat_interfaces::srv::SetTarget_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace blueboat_interfaces


#ifndef _WIN32
# define DEPRECATED__blueboat_interfaces__srv__SetTarget_Response __attribute__((deprecated))
#else
# define DEPRECATED__blueboat_interfaces__srv__SetTarget_Response __declspec(deprecated)
#endif

namespace blueboat_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetTarget_Response_
{
  using Type = SetTarget_Response_<ContainerAllocator>;

  explicit SetTarget_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit SetTarget_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    blueboat_interfaces::srv::SetTarget_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const blueboat_interfaces::srv::SetTarget_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<blueboat_interfaces::srv::SetTarget_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<blueboat_interfaces::srv::SetTarget_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      blueboat_interfaces::srv::SetTarget_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<blueboat_interfaces::srv::SetTarget_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      blueboat_interfaces::srv::SetTarget_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<blueboat_interfaces::srv::SetTarget_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<blueboat_interfaces::srv::SetTarget_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<blueboat_interfaces::srv::SetTarget_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__blueboat_interfaces__srv__SetTarget_Response
    std::shared_ptr<blueboat_interfaces::srv::SetTarget_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__blueboat_interfaces__srv__SetTarget_Response
    std::shared_ptr<blueboat_interfaces::srv::SetTarget_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetTarget_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetTarget_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetTarget_Response_

// alias to use template instance with default allocator
using SetTarget_Response =
  blueboat_interfaces::srv::SetTarget_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace blueboat_interfaces

namespace blueboat_interfaces
{

namespace srv
{

struct SetTarget
{
  using Request = blueboat_interfaces::srv::SetTarget_Request;
  using Response = blueboat_interfaces::srv::SetTarget_Response;
};

}  // namespace srv

}  // namespace blueboat_interfaces

#endif  // BLUEBOAT_INTERFACES__SRV__DETAIL__SET_TARGET__STRUCT_HPP_
