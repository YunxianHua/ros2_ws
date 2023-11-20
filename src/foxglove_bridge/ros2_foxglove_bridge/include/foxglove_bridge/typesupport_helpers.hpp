//
// Created by coffee on 11/20/23.
//

#ifndef ROS2_WS_TYPESUPPORT_HELPERS_HPP
#define ROS2_WS_TYPESUPPORT_HELPERS_HPP
#include <memory>
#include <string>
#include <tuple>

#include "rcpputils/shared_library.hpp"
#include "rosidl_runtime_cpp/message_type_support_decl.hpp"

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
/// Load the type support library for the given type.
/**
 * \param[in] type The topic type, e.g. "std_msgs/msg/String"
 * \param[in] typesupport_identifier Type support identifier, typically "rosidl_typesupport_cpp"
 * \return A shared library
 */
    RCLCPP_PUBLIC
            std::shared_ptr<rcpputils::SharedLibrary>
    get_typesupport_library(const std::string & type, const std::string & typesupport_identifier);

/// Extract the type support handle from the library.
/**
 * The library needs to match the topic type. The shared library must stay loaded for the lifetime of the result.
 * \param[in] type The topic type, e.g. "std_msgs/msg/String"
 * \param[in] typesupport_identifier Type support identifier, typically "rosidl_typesupport_cpp"
 * \param[in] library The shared type support library
 * \return A type support handle
 */
    RCLCPP_PUBLIC
    const rosidl_message_type_support_t *
    get_typesupport_handle(
            const std::string & type,
            const std::string & typesupport_identifier,
            rcpputils::SharedLibrary & library);

}  // namespace rclcpp


#endif //ROS2_WS_TYPESUPPORT_HELPERS_HPP
