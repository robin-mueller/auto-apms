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
#include "rclcpp/rclcpp.hpp"

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

  inline static const std::string LOGGER_NAME = "behavior_tree_builder";

public:
  using TreeElement = TreeDocument::TreeElement;
  using NodeElement = TreeDocument::NodeElement;

  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(TreeBuilder)

  /**
   * @brief TreeBuilder constructor.
   *
   * Arguments @p ros_node_ptr, @p tree_node_waitables_callback_group and @p tree_node_waitables_executor are passed to
   * behavior tree node plugins which utilize ROS 2 communication interfaces or waitables in general, that is, all nodes
   * inheriting from RosPublisherNode, RosSubscriberNode, RosServiceNode or RosActionNode.
   *
   * @param[in] ros_node Weak pointer to the ROS 2 node instance to be associate the behavior tree node with.
   * @param[in] tree_node_waitables_callback_group The ROS 2 callback group to be used within tree nodes when adding
   * waitables.
   * @param[in] tree_node_waitables_executor The ROS 2 executor instance that may be used for executing work provided by
   * the node's waitables.
   * @param tree_node_loader Shared pointer to the behavior tree node plugin loader instance.
   */
  TreeBuilder(
    rclcpp::Node::WeakPtr ros_node, rclcpp::CallbackGroup::WeakPtr tree_node_waitables_callback_group,
    rclcpp::executors::SingleThreadedExecutor::WeakPtr tree_node_waitables_executor,
    NodeRegistrationLoader::SharedPtr tree_node_loader = NodeRegistrationLoader::make_shared());

  /**
   * @brief TreeBuilder constructor.
   *
   * Using this signature you'll only be able to load behavior tree nodes that don't require an instance of
   * RosNodeContext during construction time.
   *
   * @param tree_node_loader Shared pointer to the behavior tree node plugin loader instance.
   */
  TreeBuilder(NodeRegistrationLoader::SharedPtr tree_node_loader = NodeRegistrationLoader::make_shared());

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

  /* Factory related member functions */

  TreeBuilder & setScriptingEnum(const std::string & enum_name, int val);

  template <typename EnumT>
  TreeBuilder & setScriptingEnumsFromType();

  /**
   * @brief Load behavior tree node plugins and register them with the internal behavior tree factory.
   *
   * This makes it possible to add any nodes specified in @p tree_node_manifest to the tree.
   *
   * @param[in] tree_node_manifest Parameters for locating and configuring the behavior tree node plugins.
   * @param[in] override If @p tree_node_manifest specifies nodes that have already been registered, unregister the
   * existing plugin and use the new one instead.
   * @throw exceptions::TreeBuildError if registration fails.
   */
  TreeBuilder & makeNodesAvailable(const NodeManifest & tree_node_manifest, bool override = false);

  std::unordered_map<std::string, BT::NodeType> getAvailableNodeTypeMap(bool include_native = true) const;

  std::set<std::string> getAvailableNodeNames(bool include_native = true) const;

  TreeBuilder & addNodeModelToDocument(bool include_native = false);

  // Verify the structure of this tree document and that all nodes are registered with the factory
  bool verify() const;

  /* Create a tree instance */

  Tree instantiate(const std::string & root_tree_name, TreeBlackboardSharedPtr bb_ptr = TreeBlackboard::create());

  Tree instantiate(const TreeElement & tree, TreeBlackboardSharedPtr bb_ptr = TreeBlackboard::create());

  Tree instantiate(TreeBlackboardSharedPtr bb_ptr = TreeBlackboard::create());

private:
  void getNodeModel(tinyxml2::XMLDocument & doc, bool include_native = false) const;

  rclcpp::Node::WeakPtr ros_node_wptr_;
  rclcpp::CallbackGroup::WeakPtr tree_node_waitables_callback_group_wptr_;
  rclcpp::executors::SingleThreadedExecutor::WeakPtr tree_node_waitables_executor_wptr_;
  NodeRegistrationLoader::SharedPtr tree_node_loader_ptr_;
  rclcpp::Logger logger_;
  TreeDocument doc_;
  BT::BehaviorTreeFactory factory_;
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