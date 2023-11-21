//
// Created by coffee on 11/20/23.
//

#ifndef ROS2_WS_GENERIC_PUBLISHER_HPP
#define ROS2_WS_GENERIC_PUBLISHER_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace rclcpp
{
    class GenericPublisher : public rclcpp::PublisherBase
    {
    public:
        RCLCPP_SMART_PTR_DEFINITIONS(GenericPublisher)

        GenericPublisher(
                rclcpp::node_interfaces::NodeBaseInterface * node_base,
                const rosidl_message_type_support_t & type_support,
                const std::string & topic_name,
                const rclcpp::QoS & qos);

        RCLCPP_PUBLIC
        virtual ~GenericPublisher() = default;

        RCLCPP_PUBLIC
        void publish(const std::shared_ptr<rcl_serialized_message_t>& message) ;
    };

}

#endif //ROS2_WS_GENERIC_PUBLISHER_HPP
