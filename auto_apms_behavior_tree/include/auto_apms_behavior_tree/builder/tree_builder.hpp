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

#include <memory>

#include "auto_apms_behavior_tree/definitions.hpp"
#include "auto_apms_behavior_tree/node/node_manifest.hpp"
#include "auto_apms_behavior_tree/resource/node_registration_loader.hpp"
#include "auto_apms_behavior_tree/resource/tree_resource.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"

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
  static inline const char ROOT_TREE_ATTRIBUTE_NAME[] = "main_tree_to_execute";
  static inline const char TREE_ELEMENT_NAME[] = "BehaviorTree";
  static inline const char TREE_NAME_ATTRIBUTE_NAME[] = "ID";

public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(TreeBuilder)

  using Document = tinyxml2::XMLDocument;
  using ElementPtr = tinyxml2::XMLElement *;
  using ConstElementPtr = const tinyxml2::XMLElement *;
  using PortValues = std::map<std::string, std::string>;

  TreeBuilder(
    rclcpp::Node::SharedPtr node_ptr,
    NodeRegistrationLoader::SharedPtr tree_node_loader_ptr = NodeRegistrationLoader::make_shared());

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

  std::unordered_map<std::string, BT::NodeType> getRegisteredNodesTypeMap() const;

  std::vector<std::string> getRegisteredNodes() const;

  TreeBuilder & mergeTreesFromDocument(const Document & doc);

  TreeBuilder & mergeTreesFromString(const std::string & tree_str);

  TreeBuilder & mergeTreesFromFile(const std::string & tree_file_path);

  TreeBuilder & mergeTreesFromResource(const TreeResource & resource);

  ElementPtr insertNewTreeElement(const std::string & tree_name);

  ElementPtr insertNewNodeElement(
    ElementPtr parent_element, const std::string & node_name, const NodeRegistrationParams & registration_params = {});

  /**
   * @brief Adds node port values to the node specified by @p node_element.
   *
   * Will use each port's default value if one is implemented and the corresponding key doesn't exist in @p port_values.
   * If the argument is omitted, only default values will be used.
   *
   * If @p verify is `true`, the method additionally verifies that @p port_values complies with the node's port model
   * and throws an exception if there are any values for unkown port names. If set to `false`, no error is raised in
   * such a case and values for unkown port names are silently discarded thus won't appear in the element's list of port
   * attributes.
   *
   * @param node_element Pointer to the node element.
   * @param port_values Port values to be used to fill the corresponding attributes of the node element.
   * @param verify Flag wether to verify that @p port_values complies with the node's port model.
   * @return Reference to the modified TreeBuilder object.
   * @throws exceptions::TreeBuildError if @p verify is set to `true` and @p port_values contains keys that are not
   * associated with the node.
   */
  TreeBuilder & addNodePortValues(ElementPtr node_element, PortValues port_values = {}, bool verify = true);

  bool isExistingTreeName(const std::string & tree_name);

  ElementPtr getTreeElement(const std::string & tree_name);

  static std::vector<std::string> getAllTreeNames(const Document & doc);

  std::vector<std::string> getAllTreeNames() const;

  bool hasRootTreeName() const;

  std::string getRootTreeName() const;

  TreeBuilder & setRootTreeName(const std::string & root_tree_name);

  static std::string writeTreeDocumentToString(const Document & doc);

  std::string writeTreeDocumentToString() const;

  // Verify the structure of this tree document and that all mentioned nodes are registered with the factory
  bool verifyTree() const;

  Tree instantiateTree(const std::string root_tree_name, TreeBlackboardSharedPtr bb_ptr = TreeBlackboard::create());

  Tree instantiateTree(TreeBlackboardSharedPtr bb_ptr = TreeBlackboard::create());

private:
  Document doc_;
  BT::BehaviorTreeFactory factory_;
  rclcpp::Node::WeakPtr node_wptr_;
  NodeRegistrationLoader::SharedPtr tree_node_loader_ptr_;
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

}  // namespace auto_apms_behavior_tree