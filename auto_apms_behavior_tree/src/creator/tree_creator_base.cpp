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

#include "auto_apms_behavior_tree/creator/tree_creator_base.hpp"

namespace auto_apms_behavior_tree
{

TreeCreatorBase::TreeCreatorBase(rclcpp::Node::SharedPtr node_ptr, TreeBuilder::SharedPtr tree_builder_ptr)
: logger_(node_ptr->get_logger()), node_wptr_(node_ptr), builder_ptr_(tree_builder_ptr)
{
}

TreeCreatorBase::TreeCreatorBase(rclcpp::Node::SharedPtr node_ptr)
: TreeCreatorBase(node_ptr, TreeBuilder::make_shared(node_ptr))
{
}

rclcpp::Node::SharedPtr TreeCreatorBase::getNodePtr() const { 
  if (node_wptr_.expired()) {
    throw std::runtime_error("TreeCreatorBase: Weak pointer to rclcpp::Node expired.");
  }
  return node_wptr_.lock();
 }

void TreeCreatorBase::configureBlackboard(TreeBlackboard & /*bb*/) {}

Tree TreeCreatorBase::createTree(const std::string & tree_name, TreeBlackboardSharedPtr bb_ptr)
{
  configureTreeBuilder(*builder_ptr_);
  configureBlackboard(*bb_ptr);
  return builder_ptr_->buildTree(tree_name, bb_ptr);
}

Tree TreeCreatorBase::createTree(TreeBlackboardSharedPtr bb_ptr) { return createTree("", bb_ptr); }

}  // namespace auto_apms_behavior_tree