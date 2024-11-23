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

#include <optional>

#include "auto_apms_behavior_tree_core/builder.hpp"
#include "auto_apms_behavior_tree_core/definitions.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"

namespace auto_apms_behavior_tree
{

/**
 * @brief Inheriting classes must use the
 * [builder](https://refactoring.guru/design-patterns/builder/cpp/example#lang-features) API offered by
 * core::TreeBuilder inside the pure virtual method buildTree() to configure a behavior tree. Implementing various
 * TreeBuildHandler classes allows TreeExecutorNode to dynamically change the way how a tree is created.
 */
class TreeBuildHandler
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(TreeBuildHandler)

  /* Convenience aliases for deriving classes */

  using TreeBlackboard = auto_apms_behavior_tree::TreeBlackboard;
  using TreeResource = core::TreeResource;
  using TreeBuilder = core::TreeBuilder;
  using TreeElement = core::TreeDocument::TreeElement;
  using NodeElement = core::TreeDocument::NodeElement;

  TreeBuildHandler(rclcpp::Node::SharedPtr node_ptr);

  virtual ~TreeBuildHandler() = default;

  virtual bool setBuildRequest(const std::string & build_request, const std::string & root_tree_name) = 0;

  virtual TreeElement buildTree(TreeBuilder & builder, TreeBlackboard & bb) = 0;

  rclcpp::Node::SharedPtr getNodePtr() const;

protected:
  const rclcpp::Logger logger_;

private:
  rclcpp::Node::WeakPtr ros_node_wptr_;
};

}  // namespace auto_apms_behavior_tree