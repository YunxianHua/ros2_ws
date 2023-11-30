//
// Created by coffee on 11/23/23.
//
#include "rclcpp/rclcpp.hpp"
#include "franka_msgs/srv/set_joint_stiffness.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <thread>
#include <memory>
#include <atomic>

struct JointSafetyRange {
    double max_value;
    double min_value;

    JointSafetyRange(double min, double max) : max_value(max), min_value(min) {}
};

static std::array<double, 7> current_joint_state{0, 0, 0, 0, 0, 0, 0};
static std::array<double, 7> dest_joint_state{0, 0, 0, 0, 0, 0, 0};

const static int move_step_count(20);
static std::atomic<bool> is_moving(false);

static JointSafetyRange joint_safety_range[7]{
        JointSafetyRange(-2.90, 2.90),
        JointSafetyRange(-1.76, 1.76),
        JointSafetyRange(-2.90, 2.90),
        JointSafetyRange(-3.07, -0.07),
        JointSafetyRange(-2.90, 2.90),
        JointSafetyRange(-0.02, 3.75),
        JointSafetyRange(-2.90, 2.90)
};

void publishJointStateThread(const std::shared_ptr<rclcpp::Node> &node) {
    rclcpp::Clock clock;
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    auto joint_states_publisher = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos_profile);
    sensor_msgs::msg::JointState joint_states;
    joint_states.name.resize(7);
    joint_states.position.resize(7);
    for (int i = 0; i < 7; ++i) {
        joint_states.name[i] = "panda_joint" + std::to_string(i + 1);
    }

    while (rclcpp::ok()){
        if (is_moving) {
            std::array<double, 7> move_step{};
            for (int i = 0; i < 7; ++i) {
                auto max = joint_safety_range[i].max_value, min = joint_safety_range[i].min_value;
                auto dest_state = std::min(dest_joint_state[i], max);
                dest_state = std::max(dest_state, min);
                move_step[i] = (dest_state - current_joint_state[i]) / move_step_count;
            }

            for (int i = 0; i < move_step_count; ++i) {
                joint_states.header.stamp = clock.now();
                for (int j = 0; j < 7; ++j) {
                    auto current_state = current_joint_state[j] + move_step[j];
                    joint_states.position[j] = current_state;
                    current_joint_state[j] = current_state;
                }
                joint_states_publisher->publish(joint_states);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
            is_moving.store(false);
        } else {
            joint_states.header.stamp = clock.now();
            joint_states_publisher->publish(joint_states);
            rclcpp::sleep_for(std::chrono::milliseconds(1000));
        }
    }
}

class FrankaDemoNode : public rclcpp::Node {
public:
    FrankaDemoNode() : Node("franka_demo_node") {

        service_ = create_service<franka_msgs::srv::SetJointStiffness>(
                "set_joint_stiffness",
                [this](const std::shared_ptr<franka_msgs::srv::SetJointStiffness::Request> request,
                       std::shared_ptr<franka_msgs::srv::SetJointStiffness::Response> response) {
                    RCLCPP_INFO(this->get_logger(), "set_joint_stiffness service was called.");

                    if (request->joint_stiffness.size() != 7) {
                        response->success = false;
                        response->error = "joint stiffness size must be 7!";
                        return;
                    }
                    is_moving.store(true);
                    dest_joint_state = request->joint_stiffness;

                    response->success = true;
                    response->error = "success!";
                });
    }

private:
    rclcpp::Service<franka_msgs::srv::SetJointStiffness>::SharedPtr service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrankaDemoNode>();

    std::thread joint_state_publisher(publishJointStateThread, node);
    joint_state_publisher.detach();
    RCLCPP_INFO(node->get_logger(), "create a joint state publish thread.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
