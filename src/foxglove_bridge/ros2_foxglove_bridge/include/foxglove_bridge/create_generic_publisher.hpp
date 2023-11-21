//
// Created by coffee on 11/20/23.
//

#ifndef ROS2_WS_CREATE_GENERIC_PUBLISHER_HPP
#define ROS2_WS_CREATE_GENERIC_PUBLISHER_HPP

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/qos.hpp"

#include "foxglove_bridge/generic_publisher.hpp"
#include "foxglove_bridge/typesupport_helpers.hpp"

namespace rclcpp
{
    std::shared_ptr<GenericPublisher> create_generic_publisher(
            rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
            const std::string & topic, const std::string & type, const rclcpp::QoS & qos)
    {
        auto library_generic_publisher = rclcpp::get_typesupport_library(
                type, "rosidl_typesupport_cpp");
        auto type_support = rclcpp::get_typesupport_handle(
                type, "rosidl_typesupport_cpp", library_generic_publisher);
        return std::make_shared<GenericPublisher>(
                topics_interface->get_node_base_interface(), *type_support, topic, qos);
    }
}  // namespace rclcpp

#endif //ROS2_WS_CREATE_GENERIC_PUBLISHER_HPP
