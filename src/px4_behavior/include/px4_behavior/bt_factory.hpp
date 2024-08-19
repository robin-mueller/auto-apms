#pragma once

#include <chrono>
#include <filesystem>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace px4_behavior {

enum class RegistrationStatus : uint8_t {
    SUCCESS = 0,
    MISSING_CONFIG,
    MISSING_PLUGIN,
};

BT::RosNodeParams CreateRosNodeParams(
    const rclcpp::Node::SharedPtr& node,
    const std::string& default_port_value,
    const std::chrono::milliseconds& request_timeout = std::chrono::milliseconds(1500),
    const std::chrono::milliseconds& wait_timeout = std::chrono::milliseconds(3000));

RegistrationStatus RegisterBTNodePlugins(BT::BehaviorTreeFactory& factory,
                                       const rclcpp::Node::SharedPtr& node,
                                       const std::filesystem::path& config_yaml,
                                       const std::vector<std::string>& build_infos = {});

}  // namespace px4_behavior
