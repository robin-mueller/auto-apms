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

#include <memory>

#include "auto_apms_behavior_tree_core/definitions.hpp"
#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include "auto_apms_behavior_tree_core/node/node_registration_loader.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_resource.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"

namespace auto_apms_behavior_tree::core
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
  friend class TreeDocument;

public:
  using TreeElement = TreeDocument::TreeElement;
  using NodeElement = TreeDocument::NodeElement;

  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(TreeBuilder)

  TreeBuilder(
    rclcpp::Node::SharedPtr node_ptr,
    NodeRegistrationLoader::SharedPtr tree_node_loader_ptr = NodeRegistrationLoader::make_shared());

  /* TreeDocument methods */

  TreeBuilder & mergeTreeDocument(const TreeDocument & other, bool adopt_root_tree = false);

  TreeElement newTree(const std::string & tree_name);

  TreeElement newTreeFromDocument(const TreeDocument & other, const std::string & tree_name = "");

  TreeElement newTreeFromString(const std::string & tree_str, const std::string & tree_name = "");

  TreeElement newTreeFromFile(const std::string & path, const std::string & tree_name = "");

  TreeElement newTreeFromResource(const TreeResource & resource, const std::string & tree_name = "");

  bool hasTree(const std::string & tree_name);

  TreeElement getTree(const std::string & tree_name);

  TreeBuilder & setRootTreeName(const std::string & tree_name);

  TreeBuilder & setRootTreeName(const TreeElement & tree);

  bool hasRootTree();

  TreeElement getRootTree();

  TreeBuilder & removeTree(const std::string & tree_name);

  TreeBuilder & removeTree(const TreeElement & tree);

  std::vector<std::string> getAllTreeNames() const;

  std::string writeTreeDocumentToString() const;

  /* Factory related member funtions */

  TreeBuilder & setScriptingEnum(const std::string & enum_name, int val);

  template <typename EnumT>
  TreeBuilder & setScriptingEnumsFromType();

  /**
   * @brief Load behavior tree node plugins and register with behavior tree factory.
   *
   * @param[in] node_manifest Parameters for locating and configuring the behavior tree node plugins.
   * @param[in] override If @p node_manifest specifies nodes that have already been registered, unregister the
   * existing plugin and use the new one instead.
   * @throw exceptions::TreeBuildError if registration fails.
   */
  TreeBuilder & loadNodePlugins(const NodeManifest & node_manifest, bool override = false);

  std::unordered_map<std::string, BT::NodeType> getRegisteredNodeTypeMap(bool include_native = false) const;

  std::set<std::string> getAvailableNodeNames(bool include_native = false) const;

  TreeBuilder & addNodeModelToDocument(bool include_native = false);

  // Verify the structure of this tree document and that all nodes are registered with the factory
  bool verify() const;

  /* Create a tree instance */

  Tree instantiate(const std::string & root_tree_name, TreeBlackboardSharedPtr bb_ptr = TreeBlackboard::create());

  Tree instantiate(const TreeElement & tree, TreeBlackboardSharedPtr bb_ptr = TreeBlackboard::create());

  Tree instantiate(TreeBlackboardSharedPtr bb_ptr = TreeBlackboard::create());

private:
  void getNodeModel(tinyxml2::XMLDocument & doc, bool include_native = false) const;

  TreeDocument doc_;
  BT::BehaviorTreeFactory factory_;
  rclcpp::Node::WeakPtr ros_node_wptr_;
  NodeRegistrationLoader::SharedPtr tree_node_loader_ptr_;
  const std::map<std::string, std::string> all_node_classes_package_map_;
  const std::set<std::string> native_node_names_;
  std::map<std::string, std::string> registered_node_class_names_map_;
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

template <typename EnumT>
TreeBuilder & TreeBuilder::setScriptingEnumsFromType()
{
  factory_.registerScriptingEnums<EnumT>();
  return *this;
}

}  // namespace auto_apms_behavior_tree::core