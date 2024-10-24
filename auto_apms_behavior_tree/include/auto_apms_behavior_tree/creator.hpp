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

#include "auto_apms_behavior_tree/node/plugin_loader.hpp"
#include "auto_apms_behavior_tree/resource.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

/**
 * @defgroup auto_apms_behavior_tree AutoAPMS - Behavior Tree
 * @brief Useful tooling for Behavior Tree development.
 */

namespace auto_apms_behavior_tree {

/**
 * @brief Central entry point for creating instances of behavior trees.
 *
 * This class wraps the functionality provided by the BT::BehaviorTreeFactory class of
 * [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) and offers a user-friendly API for creating
 * behavior trees. In contrast to the original package, the user doesn't need to manually register behavior tree node
 * entities with the factory in order to create an instance of BT::Tree. Instead, this class
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

    BTCreator(const BTNodePluginLoader& node_plugin_loader = {});

    static SharedPtr FromResource(rclcpp::Node::SharedPtr node_ptr, const BTResource& resource);

    void AddTreeFromString(const std::string& tree_str,
                           const BTNodePluginManifest& node_plugin_manifest,
                           rclcpp::Node::SharedPtr node_ptr);

    void AddTreeFromFile(const std::string& tree_file_path,
                         const BTNodePluginManifest& node_plugin_manifest,
                         rclcpp::Node::SharedPtr node_ptr);

    void AddTreeFromXMLDocument(const tinyxml2::XMLDocument& doc,
                                const BTNodePluginManifest& node_plugin_manifest,
                                rclcpp::Node::SharedPtr node_ptr);

    BT::Tree CreateTree(const std::string& main_tree_id,
                        BT::Blackboard::Ptr root_blackboard_ptr = BT::Blackboard::create());

    BT::Tree CreateMainTree(BT::Blackboard::Ptr root_blackboard_ptr = BT::Blackboard::create());

    std::string GetMainTreeName() const;

    void SetMainTreeName(const std::string& main_id);

    std::string WriteToString() const;

   private:
    static std::string WriteXMLDocumentToString(const tinyxml2::XMLDocument& doc);

   public:
    BT::BehaviorTreeFactory& factory();

   private:
    tinyxml2::XMLDocument doc_;
    BTNodePluginLoader node_plugin_loader_;
    BT::BehaviorTreeFactory factory_;
};

}  // namespace auto_apms_behavior_tree