#pragma once

#include <chrono>
#include <filesystem>
#include <optional>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace px4_behavior {

BT::RosNodeParams CreateRosNodeParams(
    const rclcpp::Node::SharedPtr& node,
    const std::string& default_port_value,
    const std::chrono::milliseconds& request_timeout = std::chrono::milliseconds(1500),
    const std::chrono::milliseconds& wait_timeout = std::chrono::milliseconds(3000));

bool RegisterBTNodePlugins(const rclcpp::Node::SharedPtr& node,
                           BT::BehaviorTreeFactory& factory,
                           const std::string& plugin_config_path,
                           const std::vector<std::string>& build_infos = {});
bool RegisterBTNodePlugins(const rclcpp::Node::SharedPtr& node,
                           BT::BehaviorTreeFactory& factory,
                           const std::set<std::string>& plugin_config_paths,
                           const std::vector<std::string>& build_infos = {});

BT::Tree CreateBehaviorTreeFromResource(const rclcpp::Node::SharedPtr& node,
                                        std::optional<const std::string> tree_file_name = std::nullopt,
                                        std::optional<const std::string> tree_id = std::nullopt,
                                        std::optional<const std::string> package_name = std::nullopt);
BT::Tree CreateBehaviorTreeFromResource(const rclcpp::Node::SharedPtr& node,
                                        BT::BehaviorTreeFactory& factory,
                                        std::optional<const std::string> tree_file_name = std::nullopt,
                                        std::optional<const std::string> tree_id = std::nullopt,
                                        std::optional<const std::string> package_name = std::nullopt);
BT::Tree CreateBehaviorTreeFromResource(const rclcpp::Node::SharedPtr& node,
                                        const BT::Blackboard::Ptr& parent_blackboard,
                                        std::optional<const std::string> tree_file_name = std::nullopt,
                                        std::optional<const std::string> tree_id = std::nullopt,
                                        std::optional<const std::string> package_name = std::nullopt);
BT::Tree CreateBehaviorTreeFromResource(const rclcpp::Node::SharedPtr& node,
                                        BT::BehaviorTreeFactory& factory,
                                        const BT::Blackboard::Ptr& parent_blackboard,
                                        std::optional<const std::string> tree_file_name = std::nullopt,
                                        std::optional<const std::string> tree_id = std::nullopt,
                                        std::optional<const std::string> package_name = std::nullopt);

}  // namespace px4_behavior
