//
// Created by coffee on 11/20/23.
//

#ifndef ROS2_WS_CREATE_GENERIC_SUBSCRIPTION_HPP
#define ROS2_WS_CREATE_GENERIC_SUBSCRIPTION_HPP
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rcl/subscription.h"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/subscription_options.hpp"

#include "foxglove_bridge/generic_subscription.hpp"
#include "foxglove_bridge/typesupport_helpers.hpp"

namespace rclcpp
{

    std::shared_ptr<GenericSubscription> create_generic_subscription(
            rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
            const std::string & topic,
            const std::string & type,
            const rclcpp::QoS & qos,
            std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback)
    {
        auto library_generic_subscriptor_ = rclcpp::get_typesupport_library(
                type, "rosidl_typesupport_cpp");
        auto type_support = rclcpp::get_typesupport_handle(
                type, "rosidl_typesupport_cpp", library_generic_subscriptor_);
        auto subscription = std::shared_ptr<GenericSubscription>();

        try {
            subscription = std::make_shared<GenericSubscription>(
                    topics_interface->get_node_base_interface(),
                    *type_support,
                    topic,
                    qos,
                    callback);

            topics_interface->add_subscription(subscription, nullptr);
        } catch (const std::runtime_error & ex) {
            std::runtime_error( "Error subscribing to topic '" + topic + "'. Error: " + ex.what());
        }

        return subscription;
    }
}  // namespace rclcpp

#endif //ROS2_WS_CREATE_GENERIC_SUBSCRIPTION_HPP
