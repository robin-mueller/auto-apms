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

#include "auto_apms_behavior_tree/creator/tree_builder.hpp"
#include "auto_apms_behavior_tree/definitions.hpp"
#include "rclcpp/node.hpp"

namespace auto_apms_behavior_tree
{

class TreeCreatorBase
{
public:
  TreeCreatorBase(rclcpp::Node::SharedPtr node_ptr);

  virtual ~TreeCreatorBase() = default;

  virtual bool setRequest(const std::string & request) = 0;

  virtual void configureTreeBuilder(TreeBuilder & builder) = 0;

  virtual void configureBlackboard(TreeBlackboard & bb);

  BT::Tree createTree(TreeBuilder & builder, TreeBlackboardSharedPtr bb_ptr);

protected:
  rclcpp::Node::SharedPtr node_ptr_;
  const rclcpp::Logger logger_;
};

}  // namespace auto_apms_behavior_tree