//
// Created by coffee on 11/23/23.
//
#include "rclcpp/rclcpp.hpp"
#include "franka_msgs/srv/set_joint_stiffness.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <iostream>
#include <thread>
#include <memory>

struct JointSafetyRange {
    double max_value;
    double min_value;

    JointSafetyRange(double min, double max) : max_value(max), min_value(min) {}
};

static std::array<double, 7> current_joint_state{0, 0, 0, 0, 0, 0, 0};

const static int move_step_count = 20;

static JointSafetyRange joint_safety_range[7]{
        JointSafetyRange(-2.90, 2.90),
        JointSafetyRange(-1.76, 1.76),
        JointSafetyRange(-2.90, 2.90),
        JointSafetyRange(-3.07, -0.07),
        JointSafetyRange(-2.90, 2.90),
        JointSafetyRange(-0.02, 3.75),
        JointSafetyRange(-2.90, 2.90)
};

void moveJointThread(const rclcpp::Logger &logger, std::array<double, 7> stiffness,
                     rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher) {
    std::array<double, 7> move_step{};

    rclcpp::Clock clock;

    for (int i = 0; i < 7; ++i) {
        auto max = joint_safety_range[i].max_value, min = joint_safety_range[i].min_value;
        auto dest_state = std::min(stiffness[i], max);
        dest_state = std::max(dest_state, min);
        move_step[i] = dest_state - current_joint_state[i];
        current_joint_state[i] = dest_state;
    }
    RCLCPP_INFO(logger, "move_step: [%lf, %lf, %lf, %lf, %lf, %lf, %lf,]", move_step[0],
                move_step[1], move_step[2], move_step[3], move_step[4],
                move_step[5], move_step[6]);

    for (int i = 0; i < move_step_count; ++i) {
        sensor_msgs::msg::JointState joint_states;
        joint_states.header.stamp = clock.now();

        for (int j = 0; j < 7; ++j) {
            joint_states.name.push_back("panda_joint" + std::to_string(j + 1));
            auto current_state =
                    current_joint_state[j] - move_step[j] / move_step_count * (move_step_count - 1 - i);
            joint_states.position.push_back(current_state);
        }
        publisher->publish(joint_states);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(logger, "current joint state: [%lf, %lf, %lf, %lf, %lf, %lf, %lf,]", current_joint_state[0],
                current_joint_state[1], current_joint_state[2], current_joint_state[3], current_joint_state[4],
                current_joint_state[5], current_joint_state[6]);
}


class FrankaDemoNode : public rclcpp::Node {
public:
    FrankaDemoNode() : Node("franka_demo_node") {
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

        joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos_profile);

        service_ = create_service<franka_msgs::srv::SetJointStiffness>(
                "set_joint_stiffness",
                [this](const std::shared_ptr<franka_msgs::srv::SetJointStiffness::Request> request,
                       std::shared_ptr<franka_msgs::srv::SetJointStiffness::Response> response) {
                    if (request->joint_stiffness.size() != 7) {
                        response->success = false;
                        response->error = "joint stiffness size must be 7!";
                        return;
                    }

                    std::thread t1(moveJointThread, this->get_logger(), request->joint_stiffness,
                                   joint_states_publisher_);
                    t1.detach();
                    response->success = true;
                    response->error = "success!";
                });
    }

private:
    rclcpp::Service<franka_msgs::srv::SetJointStiffness>::SharedPtr service_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrankaDemoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
