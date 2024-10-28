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

#include "auto_apms_behavior_tree/builder/tree_build_director_base.hpp"
#include "auto_apms_behavior_tree/exceptions.hpp"

namespace auto_apms_behavior_tree
{

TreeBuildDirectorBase::TreeBuildDirectorBase(rclcpp::Node::SharedPtr node_ptr) : node_ptr_(node_ptr)
{
}

Tree TreeBuildDirectorBase::makeTree(TreeBlackboardSharedPtr root_bb_ptr)
{
  if (!executeBuildSteps(builder_))
  {
    throw exceptions::TreeBuildError("Build director failed to make tree: executeBuildSteps() returned false.");
  }
  return builder_.getTree(root_bb_ptr);
}

rclcpp::Node::SharedPtr TreeBuildDirectorBase::getNode()
{
  return node_ptr_;
}

}  // namespace auto_apms_behavior_tree