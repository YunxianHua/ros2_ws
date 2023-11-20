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

/// Create and return a GenericPublisher.
/**
 * The returned pointer will never be empty, but this function can throw various exceptions, for
 * instance when the message's package can not be found on the AMENT_PREFIX_PATH.
 *
 * \param topics_interface NodeTopicsInterface pointer used in parts of the setup
 * \param topic_name Topic name
 * \param topic_type Topic type
 * \param qos %QoS settings
 * \param options %Publisher options.
 * Not all publisher options are currently respected, the only relevant options for this
 * publisher are `event_callbacks`, `use_default_callbacks`, and `%callback_group`.
 */
    template<typename AllocatorT = std::allocator<void>>
    std::shared_ptr<GenericPublisher> create_generic_publisher(
            rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
            const std::string & topic_name,
            const std::string & topic_type,
            const rclcpp::QoS & qos,
            const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options = (
                    rclcpp::PublisherOptionsWithAllocator<AllocatorT>()
            )
    )
    {
        auto ts_lib = rclcpp::get_typesupport_library(topic_type, "rosidl_typesupport_cpp");
        auto pub = std::make_shared<GenericPublisher>(
                topics_interface->get_node_base_interface(),
                std::move(ts_lib),
                topic_name,
                topic_type,
                qos,
                options);
        topics_interface->add_publisher(pub, options.callback_group);
        return pub;
    }

}  // namespace rclcpp


#endif //ROS2_WS_CREATE_GENERIC_PUBLISHER_HPP
