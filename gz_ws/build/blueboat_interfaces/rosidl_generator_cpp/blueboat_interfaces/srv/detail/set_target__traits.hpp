// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from blueboat_interfaces:srv/SetTarget.idl
// generated code does not contain a copyright notice

#ifndef BLUEBOAT_INTERFACES__SRV__DETAIL__SET_TARGET__TRAITS_HPP_
#define BLUEBOAT_INTERFACES__SRV__DETAIL__SET_TARGET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "blueboat_interfaces/srv/detail/set_target__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace blueboat_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetTarget_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetTarget_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetTarget_Request & msg, bool use_flow_style = false)
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

}  // namespace blueboat_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use blueboat_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const blueboat_interfaces::srv::SetTarget_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  blueboat_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use blueboat_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const blueboat_interfaces::srv::SetTarget_Request & msg)
{
  return blueboat_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<blueboat_interfaces::srv::SetTarget_Request>()
{
  return "blueboat_interfaces::srv::SetTarget_Request";
}

template<>
inline const char * name<blueboat_interfaces::srv::SetTarget_Request>()
{
  return "blueboat_interfaces/srv/SetTarget_Request";
}

template<>
struct has_fixed_size<blueboat_interfaces::srv::SetTarget_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<blueboat_interfaces::srv::SetTarget_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<blueboat_interfaces::srv::SetTarget_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace blueboat_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetTarget_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetTarget_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetTarget_Response & msg, bool use_flow_style = false)
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

}  // namespace blueboat_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use blueboat_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const blueboat_interfaces::srv::SetTarget_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  blueboat_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use blueboat_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const blueboat_interfaces::srv::SetTarget_Response & msg)
{
  return blueboat_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<blueboat_interfaces::srv::SetTarget_Response>()
{
  return "blueboat_interfaces::srv::SetTarget_Response";
}

template<>
inline const char * name<blueboat_interfaces::srv::SetTarget_Response>()
{
  return "blueboat_interfaces/srv/SetTarget_Response";
}

template<>
struct has_fixed_size<blueboat_interfaces::srv::SetTarget_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<blueboat_interfaces::srv::SetTarget_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<blueboat_interfaces::srv::SetTarget_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<blueboat_interfaces::srv::SetTarget>()
{
  return "blueboat_interfaces::srv::SetTarget";
}

template<>
inline const char * name<blueboat_interfaces::srv::SetTarget>()
{
  return "blueboat_interfaces/srv/SetTarget";
}

template<>
struct has_fixed_size<blueboat_interfaces::srv::SetTarget>
  : std::integral_constant<
    bool,
    has_fixed_size<blueboat_interfaces::srv::SetTarget_Request>::value &&
    has_fixed_size<blueboat_interfaces::srv::SetTarget_Response>::value
  >
{
};

template<>
struct has_bounded_size<blueboat_interfaces::srv::SetTarget>
  : std::integral_constant<
    bool,
    has_bounded_size<blueboat_interfaces::srv::SetTarget_Request>::value &&
    has_bounded_size<blueboat_interfaces::srv::SetTarget_Response>::value
  >
{
};

template<>
struct is_service<blueboat_interfaces::srv::SetTarget>
  : std::true_type
{
};

template<>
struct is_service_request<blueboat_interfaces::srv::SetTarget_Request>
  : std::true_type
{
};

template<>
struct is_service_response<blueboat_interfaces::srv::SetTarget_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // BLUEBOAT_INTERFACES__SRV__DETAIL__SET_TARGET__TRAITS_HPP_
