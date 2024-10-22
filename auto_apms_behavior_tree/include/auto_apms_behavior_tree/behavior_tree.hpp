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
 */

/**
 * @brief Struct containing behavior tree resource data
 * @ingroup auto_apms_behavior_tree
 */
struct BTResource
{
   private:
    // The default constructor is private. To create an instance, use one of the static construction methods instead.
    BTResource() = default;

   public:
    std::string name;
    std::string tree_path;
    std::string package_name;
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

/**
 * @brief Central entry point for creating instances of behavior trees.
 *
 * This class wraps the functionality provided by the BT::BehaviorTreeFactory class of
 * [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) and offers a user-friendly API for creating
 * behavior trees. In contrast to the original package, the user doesn't need to manually register behavior tree node
 * entities with the factory in order to create an instance of BT::Tree and execute it. Instead, this class
 * conveniently automates this process for you by querying the available `ament_index` plugin resources implemented and
 * registered by the developer.
 *
 * @ingroup auto_apms_behavior_tree
 */
class BTCreator
{
    static inline const std::string MAIN_TREE_ATTRIBUTE_NAME = "main_tree_to_execute";

   public:
    using SharedPtr = std::shared_ptr<BTCreator>;
    using NodePluginManifest = BTNodePluginManifest;

    BTCreator(const std::string& file_path, const NodePluginManifest& node_plugin_manifest = {});
    BTCreator(const BTResource& resource);

    static SharedPtr FromTreeID(const std::string& tree_id, const std::string& package_name = "");
    static SharedPtr FromTreeFileName(const std::string& file_name, const std::string& package_name = "");

    static BT::Tree Create(const std::string& tree_str,
                           const std::string& main_id,
                           BT::BehaviorTreeFactory& factory,
                           BT::Blackboard::Ptr parent_blackboard_ptr = nullptr);

    BT::Tree Create(rclcpp::Node::SharedPtr node_ptr,
                    BT::BehaviorTreeFactory& factory,
                    BT::Blackboard::Ptr parent_blackboard_ptr = nullptr) const;

    BT::Tree Create(rclcpp::Node::SharedPtr node_ptr, BT::Blackboard::Ptr parent_blackboard_ptr = nullptr) const;

    std::string GetMainTreeID() const;

    void SetMainTreeID(const std::string& main_id);

    std::string WriteToString() const;

   private:
    NodePluginManifest node_plugin_manifest_;
    tinyxml2::XMLDocument doc_;
};

}  // namespace auto_apms_behavior_tree