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

TreeCreatorBase::TreeCreatorBase(rclcpp::Node::SharedPtr node_ptr)
: node_ptr_(node_ptr), logger_{node_ptr->get_logger()}
{
}

void TreeCreatorBase::configureBlackboard(TreeBlackboard & /*bb*/) {}

BT::Tree TreeCreatorBase::createTree(TreeBuilder & builder, TreeBlackboardSharedPtr bb_ptr)
{
  return builder.buildTree(bb_ptr);
}

}  // namespace auto_apms_behavior_tree