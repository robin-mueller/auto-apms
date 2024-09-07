// Copyright 2024 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
