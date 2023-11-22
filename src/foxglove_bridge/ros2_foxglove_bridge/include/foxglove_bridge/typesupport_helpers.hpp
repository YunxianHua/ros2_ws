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

namespace foxglove_bridge {
    RCLCPP_PUBLIC
    std::shared_ptr<rcpputils::SharedLibrary>
    get_typesupport_library(const std::string &type, const std::string &typesupport_identifier);

    RCLCPP_PUBLIC
    const rosidl_message_type_support_t *
    get_typesupport_handle(
            const std::string &type,
            const std::string &typesupport_identifier,
            std::shared_ptr<rcpputils::SharedLibrary> library);

    RCLCPP_PUBLIC
    std::tuple<std::string, std::string, std::string>
    extract_type_identifier(const std::string &full_type);
}  // namespace foxglove_bridge


#endif //ROS2_WS_TYPESUPPORT_HELPERS_HPP
