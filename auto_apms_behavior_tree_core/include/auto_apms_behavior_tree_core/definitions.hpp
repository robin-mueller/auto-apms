// Copyright 2024 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <map>
#include <string>
#include <vector>

#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/bt_factory.h"

/**
 * @defgroup auto_apms_behavior_tree Behavior Tree
 * @brief Powerful tooling for incorporating behavior trees for task development.
 */

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Powerful tooling for incorporating behavior trees for task development.
 */
namespace auto_apms_behavior_tree
{

using Tree = BT::Tree;
using TreeBlackboard = BT::Blackboard;
using TreeBlackboardSharedPtr = std::shared_ptr<TreeBlackboard>;
using TreeConstructor = std::function<Tree(TreeBlackboardSharedPtr)>;

/**
 * @brief Implementation details of a single data port.
 */
struct NodePortInfo
{
  /// Name of the port.
  std::string port_name;
  /// String representation of the C++ type given to the port.
  std::string port_type;
  /// Default value of the port encoded as string.
  std::string port_default;
  /// Flag whether the port implements a default value or not.
  bool port_has_default;
  /// Description of the port.
  std::string port_description;
  /// Direction of the port.
  BT::PortDirection port_direction;
};

/**
 * @brief Data structure encapsulating the information of all ports implemented by a behavior tree node.
 */
struct NodeModel
{
  /// Abstract type of the node.
  BT::NodeType type;
  /// Vector of implementation details for each data port.
  std::vector<NodePortInfo> port_infos;
};

/// Mapping of node registration names and their implementation details.
using NodeModelMap = std::map<std::string, NodeModel>;

}  // namespace auto_apms_behavior_tree
