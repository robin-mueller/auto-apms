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

#include "auto_apms_behavior_tree/builder/tree_builder.hpp"
#include "rclcpp/node.hpp"

namespace auto_apms_behavior_tree
{

class TreeBuildDirectorBase
{
public:
  TreeBuildDirectorBase(rclcpp::Node::SharedPtr node_ptr);

  virtual ~TreeBuildDirectorBase() = default;

  virtual bool setRequestedTreeIdentity(const std::string& identity) = 0;

  virtual Tree makeTree(TreeBlackboardSharedPtr root_bb_ptr);

  rclcpp::Node::SharedPtr getNodePtr();

  const rclcpp::Logger& getLogger();

private:
  virtual bool executeBuildSteps(TreeBuilder& builder) = 0;

  rclcpp::Node::SharedPtr node_ptr_;
  const rclcpp::Logger logger_;
  TreeBuilder builder_;
};

}  // namespace auto_apms_behavior_tree