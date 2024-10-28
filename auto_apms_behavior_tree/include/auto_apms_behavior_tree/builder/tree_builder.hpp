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
#include "behaviortree_cpp/bt_factory.h"
#include "auto_apms_behavior_tree/node/plugin_manifest.hpp"
#include "auto_apms_behavior_tree/resource/node_class_loader.hpp"
#include "auto_apms_behavior_tree/resource/tree_resource.hpp"

namespace auto_apms_behavior_tree
{

using Tree = BT::Tree;
using TreeBlackboard = BT::Blackboard;
using TreeBlackboardSharedPtr = std::shared_ptr<TreeBlackboard>;

/**
 * @brief Used for creating instances of BT::BehaviorTreeFactory.
 *
 * This class wraps the functionality provided by the BT::BehaviorTreeFactory class of
 * [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) and offers a user-friendly API for configuring
 * behavior trees. In contrast to the original package, the user doesn't need to manually register behavior tree node
 * entities with the factory in order to create an instance of BT::Tree. Instead, this class
 * conveniently automates this process for you by querying the available `ament_index` plugin resources implemented and
 * registered by the developer.
 *
 * @ingroup auto_apms_behavior_tree
 */
class TreeBuilder
{
  using Impl = BT::BehaviorTreeFactory;

  static inline const std::string MAIN_TREE_ATTRIBUTE_NAME = "main_tree_to_execute";
  static inline const std::string TREE_ELEMENT_NAME = "BehaviorTree";
  static inline const std::string TREE_ID_ATTRIBUTE_NAME = "ID";

public:
  TreeBuilder();

  /**
   * @brief Load behavior tree node plugins and register with behavior tree factory.
   *
   * @param[in] node_ptr ROS2 node to pass to RosNodeParams.
   * @param[in] node_plugin_manifest Parameters for locating and configuring the behavior tree node plugins.
   * @param[in] override If @p node_plugin_manifest specifies nodes that have already been registered, unregister the
   * existing plugin and use the new one instead.
   * @param[in] node_plugin_loader_ptr Shared pointer to loader for behavior tree plugin classes.
   * @throw exceptions::TreeBuildError if registration fails.
   */
  TreeBuilder& RegisterNodePlugins(
      rclcpp::Node::SharedPtr node_ptr, const BTNodePluginManifest& node_plugin_manifest, bool override = false,
      std::shared_ptr<BTNodePluginClassLoader> node_plugin_loader_ptr = MakeBTNodePluginClassLoader());

  TreeBuilder& AddTreeFromXMLDocument(const tinyxml2::XMLDocument& doc);

  TreeBuilder& AddTreeFromString(const std::string& tree_str);

  TreeBuilder& AddTreeFromFile(const std::string& tree_file_path);

  TreeBuilder& addTreeFromResource(const TreeResource& resource, rclcpp::Node::SharedPtr node_ptr);

  std::string GetMainTreeName() const;

  TreeBuilder& SetMainTreeName(const std::string& main_tree_name);

  std::string WriteTreeBufferToString() const;

  std::unordered_map<std::string, BT::NodeType> GetRegisteredNodes();

  Tree getTree(const std::string main_tree_name, TreeBlackboardSharedPtr root_bb_ptr = TreeBlackboard::create());

  Tree getTree(TreeBlackboardSharedPtr root_bb_ptr = TreeBlackboard::create());

  /* Static helper functions */

  static std::set<std::string> GetTreeNames(const tinyxml2::XMLDocument& doc);

  static std::string WriteXMLDocumentToString(const tinyxml2::XMLDocument& doc);

private:
  tinyxml2::XMLDocument doc_;
  Impl impl_;
};

}  // namespace auto_apms_behavior_tree