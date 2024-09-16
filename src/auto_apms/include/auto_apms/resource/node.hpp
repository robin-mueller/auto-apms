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

#include <map>
#include <optional>
#include <set>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "class_loader/class_loader.hpp"
#include "rclcpp/node.hpp"
#include "yaml-cpp/node/convert.h"

namespace auto_apms {
namespace resource {

/// @brief Struct holding the user defined configuration parameters for the registration of a behavior tree node plugin
struct BTNodeRegistrationConfig
{
    // Required
    std::string class_name;

    // Optional
    std::optional<std::string> port;
    std::optional<std::string> package;
    std::optional<std::string> library;
    std::optional<double> wait_timeout;     // in seconds
    std::optional<double> request_timeout;  // in seconds

    bool IsROSSpecific() const;
    BT::RosNodeParams CreateROSNodeParams(rclcpp::Node::SharedPtr node_ptr) const;
    static std::set<std::string> GetRequiredNames();
};

/**
 * @brief Map containing the information about what behavior tree nodes to register and how to load and configure
 * them.
 *
 * The map's keys are the node's registration names and the associated values
 * refer to the registration parameters of the respective node.
 */
using BTNodeRegistrationConfigMap = std::map<std::string, BTNodeRegistrationConfig>;

/// @brief Struct for behavior tree node plugin resources
struct BTNodeResource
{
    std::string class_name;
    std::string library_path;
};

/**
 * @brief Struct holding the inferred registration manifest of behavior tree node plugins.
 *
 * Configures exactly how to load and register plugins using auto_apms::RegisterBTNodePlugins.
 */
struct BTNodeRegistrationManifest
{
    std::string library_path;
    std::string package_name;  // Empty if library path was provided manually
    BTNodeRegistrationConfig params;
};

/**
 * @brief Map containing the information about what behavior tree node plugins are registered.
 *
 * The map's keys are the node's registration names and the associated values
 * refer to the registration manifest of the respective node.
 */
using BTNodeRegistrationManifestMap = std::map<std::string, BTNodeRegistrationManifest>;

BTNodeRegistrationConfigMap ParseBTNodePluginConfiguration(const std::string& path);
BTNodeRegistrationConfigMap ParseBTNodePluginConfiguration(const std::vector<std::string>& paths);

std::vector<BTNodeResource> FetchBTNodePluginResources(const std::string& package_name);

BTNodeRegistrationManifestMap CreateNodeRegistrationManifest(const BTNodeRegistrationConfigMap& registration_config);

}  // namespace resource

bool RegisterBTNodePlugins(rclcpp::Node::SharedPtr node_ptr,
                           BT::BehaviorTreeFactory& factory,
                           const std::string& plugin_config_path);
bool RegisterBTNodePlugins(rclcpp::Node::SharedPtr node_ptr,
                           BT::BehaviorTreeFactory& factory,
                           const std::vector<std::string>& plugin_config_paths);
bool RegisterBTNodePlugins(rclcpp::Node::SharedPtr node_ptr,
                           BT::BehaviorTreeFactory& factory,
                           const resource::BTNodeRegistrationManifestMap& manifest_map);

}  // namespace auto_apms

namespace YAML {

template <>
struct convert<auto_apms::resource::BTNodeRegistrationConfigMap>
{
    static Node encode(const auto_apms::resource::BTNodeRegistrationConfigMap& rhs);
    static bool decode(const Node& node, auto_apms::resource::BTNodeRegistrationConfigMap& lhs);
};

}  // namespace YAML
