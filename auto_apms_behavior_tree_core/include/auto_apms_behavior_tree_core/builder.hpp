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
class TreeBuilder : public TreeDocument
{
public:
  inline static const std::string LOGGER_NAME = "tree_builder";

  RCLCPP_SMART_PTR_DEFINITIONS(TreeBuilder)

  /**
   * @brief TreeBuilder constructor.
   *
   * Arguments @p ros_node, @p tree_node_waitables_callback_group and @p tree_node_waitables_executor are passed to
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
  explicit TreeBuilder(
    rclcpp::Node::SharedPtr ros_node, rclcpp::CallbackGroup::SharedPtr tree_node_waitables_callback_group,
    rclcpp::executors::SingleThreadedExecutor::SharedPtr tree_node_waitables_executor,
    NodeRegistrationLoader::SharedPtr tree_node_loader = NodeRegistrationLoader::make_shared());

  /**
   * @brief TreeBuilder constructor.
   *
   * Using this signature you'll only be able to instantiate behavior trees tha only contain nodes which don't require
   * an instance of RosNodeContext during construction time (non-ROS nodes).
   *
   * @param tree_node_loader Shared pointer to the behavior tree node plugin loader instance.
   */
  explicit TreeBuilder(NodeRegistrationLoader::SharedPtr tree_node_loader = NodeRegistrationLoader::make_shared());

  virtual ~TreeBuilder() override = default;

  /// @copydoc TreeDocument::registerNodes(const NodeManifest & tree_node_manifest, bool override)
  TreeBuilder & registerNodes(const NodeManifest & tree_node_manifest, bool override = false) override;

  /* Factory related member functions */

  TreeBuilder & setScriptingEnum(const std::string & enum_name, int val);

  template <typename EnumT>
  TreeBuilder & setScriptingEnumsFromType();

  /* Create a tree instance */

  Tree instantiate(const std::string & root_tree_name, TreeBlackboardSharedPtr bb_ptr = TreeBlackboard::create());

  Tree instantiate(TreeBlackboardSharedPtr bb_ptr = TreeBlackboard::create());
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