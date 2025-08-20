// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from blueboat_interfaces:srv/SetTarget.idl
// generated code does not contain a copyright notice

#ifndef BLUEBOAT_INTERFACES__SRV__DETAIL__SET_TARGET__BUILDER_HPP_
#define BLUEBOAT_INTERFACES__SRV__DETAIL__SET_TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "blueboat_interfaces/srv/detail/set_target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace blueboat_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetTarget_Request_y
{
public:
  explicit Init_SetTarget_Request_y(::blueboat_interfaces::srv::SetTarget_Request & msg)
  : msg_(msg)
  {}
  ::blueboat_interfaces::srv::SetTarget_Request y(::blueboat_interfaces::srv::SetTarget_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::blueboat_interfaces::srv::SetTarget_Request msg_;
};

class Init_SetTarget_Request_x
{
public:
  Init_SetTarget_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetTarget_Request_y x(::blueboat_interfaces::srv::SetTarget_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_SetTarget_Request_y(msg_);
  }

private:
  ::blueboat_interfaces::srv::SetTarget_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::blueboat_interfaces::srv::SetTarget_Request>()
{
  return blueboat_interfaces::srv::builder::Init_SetTarget_Request_x();
}

}  // namespace blueboat_interfaces


namespace blueboat_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetTarget_Response_accepted
{
public:
  Init_SetTarget_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::blueboat_interfaces::srv::SetTarget_Response accepted(::blueboat_interfaces::srv::SetTarget_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return std::move(msg_);
  }

private:
  ::blueboat_interfaces::srv::SetTarget_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::blueboat_interfaces::srv::SetTarget_Response>()
{
  return blueboat_interfaces::srv::builder::Init_SetTarget_Response_accepted();
}

}  // namespace blueboat_interfaces

#endif  // BLUEBOAT_INTERFACES__SRV__DETAIL__SET_TARGET__BUILDER_HPP_
