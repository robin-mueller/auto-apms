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

#include "auto_apms_behavior_tree/node/plugin_base.hpp"
#include "auto_apms_behavior_tree/node/plugin_manifest.hpp"
#include "auto_apms_behavior_tree/resource/resource.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/node.hpp"

/**
 * @defgroup auto_apms_behavior_tree AutoAPMS - Behavior Tree
 * @brief Useful tooling for Behavior Tree development.
 */

namespace auto_apms_behavior_tree {

using BTNodePluginClassLoader = pluginlib::ClassLoader<BTNodePluginBase>;

/**
 * @brief Create an instance of pluginlib::ClassLoader specifically for loading installed behavior tree node plugin
 * resources.
 * @ingroup auto_apms_behavior_tree
 * @param package_names Packages to consider when searching for behavior tree node plugin resources. Leave empty to
 * search in all packages.
 * @return pluginlib::ClassLoader object.
 */
std::shared_ptr<BTNodePluginClassLoader> MakeBTNodePluginClassLoader(const std::set<std::string>& package_names = {});

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
    using Factory = BT::BehaviorTreeFactory;

    /**
     * @brief Manually create an instance of BTCreator.
     * @param factory_ptr Shared pointer to the behavior tree factory that the trees and node plugins will register
     * with.
     * @param package_names Set of packages to consider when searching the installed behavior tree node plugin
     * resources. Leave empty to search in all packages.
     */
    BTCreator(std::shared_ptr<Factory> factory_ptr = std::make_shared<Factory>(),
              std::shared_ptr<BTNodePluginClassLoader> node_plugin_loader_ptr = MakeBTNodePluginClassLoader());

    /**
     * @brief Load behavior tree node plugins and register with behavior tree factory.
     *
     * @param[in,out] node_ptr ROS2 node to pass to RosNodeParams.
     * @param[in] node_plugin_manifest Parameters for locating and configuring the behavior tree node plugins.
     * @throw exceptions::NodeRegistrationError if registration fails.
     */
    BTCreator& RegisterNodePlugins(rclcpp::Node::SharedPtr node_ptr, const BTNodePluginManifest& node_plugin_manifest);

    static std::unordered_map<std::string, BT::NodeType> GetRegisteredNodes(const Factory& factory);

    BTCreator& AddTreeFromXMLDocument(const tinyxml2::XMLDocument& doc);

    BTCreator& AddTreeFromString(const std::string& tree_str);

    BTCreator& AddTreeFromFile(const std::string& tree_file_path);

    BTCreator& AddTreeFromResource(const BTResource& resource, rclcpp::Node::SharedPtr node_ptr);

    BT::Tree CreateTree(const std::string& main_tree_id,
                        BT::Blackboard::Ptr root_blackboard_ptr = BT::Blackboard::create());

    BT::Tree CreateMainTree(BT::Blackboard::Ptr root_blackboard_ptr = BT::Blackboard::create());

    std::string GetMainTreeName() const;

    BTCreator& SetMainTreeName(const std::string& main_id);

    std::string WriteToString() const;

   protected:
    static std::string WriteXMLDocumentToString(const tinyxml2::XMLDocument& doc);

    tinyxml2::XMLDocument doc_;
    std::shared_ptr<Factory> factory_ptr_;
    std::shared_ptr<BTNodePluginClassLoader> node_plugin_loader_ptr_;
};

}  // namespace auto_apms_behavior_tree