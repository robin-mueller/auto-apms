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

#include "auto_apms_behavior_tree_core/builder.hpp"
#include "auto_apms_behavior_tree_core/definitions.hpp"
#include "auto_apms_behavior_tree_core/node/node_registration_loader.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"

// Include all built in node models for convenience
#include "auto_apms_behavior_tree/builtin_nodes.hpp"
#include "auto_apms_behavior_tree/util/node.hpp"
#include "auto_apms_behavior_tree_core/native_nodes.hpp"

// Include exceptions if derived build handlers need to throw an TreeBuildHandlerError
#include "auto_apms_behavior_tree/exceptions.hpp"

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

  using NodeLoader = core::NodeRegistrationLoader;
  using NodeManifest = core::NodeManifest;
  using TreeResource = core::TreeResource;
  using TreeDocument = core::TreeDocument;
  using TreeBuilder = core::TreeBuilder;
  using TreeBlackboard = auto_apms_behavior_tree::TreeBlackboard;

  TreeBuildHandler(
    const std::string & name, rclcpp::Node::SharedPtr ros_node_ptr, NodeLoader::SharedPtr tree_node_loader_ptr);

  TreeBuildHandler(rclcpp::Node::SharedPtr ros_node_ptr, NodeLoader::SharedPtr tree_node_loader_ptr);

  virtual ~TreeBuildHandler() = default;

  virtual bool setBuildRequest(
    const std::string & build_request, const NodeManifest & node_manifest, const std::string & root_tree_name) = 0;

  virtual TreeDocument::TreeElement buildTree(TreeBuilder & builder, TreeBlackboard & bb) = 0;

  rclcpp::Node::SharedPtr getRosNodePtr() const;

  NodeLoader::SharedPtr getNodeLoaderPtr() const;

protected:
  const rclcpp::Logger logger_;

private:
  rclcpp::Node::WeakPtr ros_node_wptr_;
  NodeLoader::SharedPtr tree_node_loader_ptr;
};

}  // namespace auto_apms_behavior_tree