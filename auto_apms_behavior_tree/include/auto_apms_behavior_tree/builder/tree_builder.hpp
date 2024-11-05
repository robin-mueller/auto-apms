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

#include "auto_apms_behavior_tree/node/node_manifest.hpp"
#include "auto_apms_behavior_tree/resource/node_registration_class_loader.hpp"
#include "auto_apms_behavior_tree/resource/tree_resource.hpp"
#include "auto_apms_behavior_tree/definitions.hpp"

namespace auto_apms_behavior_tree
{

/**
 * @brief Class for creating behavior trees according to the builder design pattern.
 *
 * This class extends the functionality provided by BT::BehaviorTreeFactory of
 * [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) and offers a user-friendly API for configuring
 * behavior trees. In contrast to the original package, the user doesn't need to manually register behavior tree node
 * entities with the factory in order to create an instance of BT::Tree. Instead, this class
 * conveniently automates this process for you by querying the available `ament_index` plugin resources implemented and
 * registered by the developer. Additionally, this class allows for building custom XML definitions of trees
 * programmatically.
 *
 * @ingroup auto_apms_behavior_tree
 */
class TreeBuilder
{
  static inline const std::string MAIN_TREE_ATTRIBUTE_NAME = "main_tree_to_execute";
  static inline const std::string TREE_ELEMENT_NAME = "BehaviorTree";
  static inline const std::string TREE_ID_ATTRIBUTE_NAME = "ID";

public:
  TreeBuilder(std::shared_ptr<BT::BehaviorTreeFactory> factory_ptr = std::make_shared<BT::BehaviorTreeFactory>());

  /**
   * @brief Load behavior tree node plugins and register with behavior tree factory.
   *
   * @param[in] node_ptr ROS2 node to pass to RosNodeContext.
   * @param[in] node_manifest Parameters for locating and configuring the behavior tree node plugins.
   * @param[in] tree_node_loader Reference to loader for behavior tree node plugin classes.
   * @param[in] override If @p node_manifest specifies nodes that have already been registered, unregister the
   * existing plugin and use the new one instead.
   * @throw exceptions::TreeBuildError if registration fails.
   */
  TreeBuilder& registerNodePlugins(rclcpp::Node::SharedPtr node_ptr, const NodeManifest& node_manifest,
                                   NodeRegistrationClassLoader& tree_node_loader, bool override = false);

  /**
   * @overload
   *
   * Creates a default behavior tree node plugin class loader.
   */
  TreeBuilder& registerNodePlugins(rclcpp::Node::SharedPtr node_ptr, const NodeManifest& node_manifest,
                                   bool override = false);

  TreeBuilder& addTreeFromXMLDocument(const tinyxml2::XMLDocument& doc);

  TreeBuilder& addTreeFromString(const std::string& tree_str);

  TreeBuilder& addTreeFromFile(const std::string& tree_file_path);

  TreeBuilder& addTreeFromResource(const TreeResource& resource, rclcpp::Node::SharedPtr node_ptr);

  std::string getMainTreeName() const;

  TreeBuilder& setMainTreeName(const std::string& main_tree_name);

  std::string writeTreeXMLToString() const;

  std::unordered_map<std::string, BT::NodeType> getRegisteredNodes();

  Tree buildTree(const std::string main_tree_name, TreeBlackboardSharedPtr root_bb_ptr = TreeBlackboard::create());

  Tree buildTree(TreeBlackboardSharedPtr root_bb_ptr = TreeBlackboard::create());

  /* Static helper functions */

  static std::vector<std::string> getTreeNames(const tinyxml2::XMLDocument& doc);

  static std::string writeXMLDocumentToString(const tinyxml2::XMLDocument& doc);

private:
  tinyxml2::XMLDocument doc_;
  std::shared_ptr<BT::BehaviorTreeFactory> factory_ptr_;
};

}  // namespace auto_apms_behavior_tree