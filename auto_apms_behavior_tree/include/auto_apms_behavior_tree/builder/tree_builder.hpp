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
  static inline const char ROOT_ELEMENT_NAME[] = "root";
  static inline const char MAIN_TREE_ATTRIBUTE_NAME[] = "main_tree_to_execute";
  static inline const char TREE_ELEMENT_NAME[] = "BehaviorTree";
  static inline const char TREE_NAME_ATTRIBUTE_NAME[] = "ID";

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
  TreeBuilder& loadNodePlugins(rclcpp::Node::SharedPtr node_ptr, const NodeManifest& node_manifest,
                               NodeRegistrationClassLoader& tree_node_loader, bool override = false);

  /**
   * @overload
   *
   * Creates a default NodeRegistrationClassLoader.
   */
  TreeBuilder& loadNodePlugins(rclcpp::Node::SharedPtr node_ptr, const NodeManifest& node_manifest,
                               bool override = false);

  TreeBuilder& mergeTreesFromDocument(const tinyxml2::XMLDocument& doc);

  TreeBuilder& mergeTreesFromString(const std::string& tree_str);

  TreeBuilder& mergeTreesFromFile(const std::string& tree_file_path);

  TreeBuilder& mergeTreesFromResource(const TreeResource& resource, rclcpp::Node::SharedPtr node_ptr);

  bool isExistingTreeName(const std::string& tree_name);

  tinyxml2::XMLElement* insertNewTreeElement(const std::string& tree_name);

  tinyxml2::XMLElement* getTreeElement(const std::string& tree_name);

  static std::vector<std::string> getAllTreeNames(const tinyxml2::XMLDocument& doc);

  std::vector<std::string> getAllTreeNames() const;

  std::string getMainTreeName() const;

  TreeBuilder& setMainTreeName(const std::string& main_tree_name);

  static std::string writeTreeDocumentToString(const tinyxml2::XMLDocument& doc);

  std::string writeTreeDocumentToString() const;

  std::unordered_map<std::string, BT::NodeType> getRegisteredNodes() const;

  // Verify the structure of this tree document and that all mentioned nodes are registered with the factory
  bool verifyTreeDocument() const;

  Tree buildTree(const std::string main_tree_name, TreeBlackboardSharedPtr root_bb_ptr = TreeBlackboard::create());

  Tree buildTree(TreeBlackboardSharedPtr root_bb_ptr = TreeBlackboard::create());

private:
  tinyxml2::XMLDocument doc_;
  std::shared_ptr<BT::BehaviorTreeFactory> factory_ptr_;
};

}  // namespace auto_apms_behavior_tree