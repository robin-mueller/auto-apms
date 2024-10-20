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

#include <tinyxml2.h>

#include "auto_apms_behavior_tree/node_plugin_manifest.hpp"
#include "behaviortree_cpp/bt_factory.h"

namespace auto_apms_behavior_tree {

/**
 * @defgroup auto_apms_behavior_tree AutoAPMS - Behavior Tree
 * @brief Useful tooling for Behavior Tree development.
 * @{
 */

/// @brief Struct for behavior tree resource data
struct BTResource
{
    std::string name;
    std::string tree_path;
    std::string node_manifest_path;
    std::set<std::string> tree_ids;

    /**
     * @brief Collect all behavior tree resources registered by a certain package.
     * @param package_name Name of the package to search for resources.
     * @return Collection of all resources found in @p package_name.
     */
    static std::vector<BTResource> CollectFromPackage(const std::string& package_name);

    static BTResource SelectByID(const std::string& tree_id, const std::string& package_name = "");

    static BTResource SelectByFileName(const std::string& file_name, const std::string& package_name = "");
};

class BehaviorTree
{
    static inline const std::string MAIN_TREE_ATTRIBUTE_NAME = "main_tree_to_execute";

   public:
    using NodePluginManifest = BTNodePluginManifest;

    BehaviorTree(const std::string& file_path, const NodePluginManifest& node_plugin_manifest = {});
    BehaviorTree(const BTResource& resource);

    static BT::Tree Create(const std::string& tree_str,
                           const std::string& main_id,
                           BT::BehaviorTreeFactory& factory,
                           BT::Blackboard::Ptr parent_blackboard_ptr = nullptr);

    static BT::Tree Create(rclcpp::Node::SharedPtr node_ptr,
                           const BTResource& resource,
                           const std::string& main_id,
                           BT::BehaviorTreeFactory& factory,
                           BT::Blackboard::Ptr parent_blackboard_ptr = nullptr);

    static BT::Tree Create(rclcpp::Node::SharedPtr node_ptr,
                           const BTResource& resource,
                           const std::string& main_id,
                           BT::Blackboard::Ptr parent_blackboard_ptr = nullptr);

    BT::Tree Create(rclcpp::Node::SharedPtr node_ptr,
                    BT::BehaviorTreeFactory& factory,
                    BT::Blackboard::Ptr parent_blackboard_ptr = nullptr) const;

    BT::Tree Create(rclcpp::Node::SharedPtr node_ptr, BT::Blackboard::Ptr parent_blackboard_ptr = nullptr) const;

    std::string GetMainID() const;

    BehaviorTree& SetMainID(const std::string& main_id);

    std::string WriteToString() const;

   private:
    NodePluginManifest node_plugin_manifest_;
    tinyxml2::XMLDocument doc_;
};
/// @}

}  // namespace auto_apms_behavior_tree