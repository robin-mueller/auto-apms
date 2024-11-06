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

#include "auto_apms_behavior_tree/node.hpp"
#include "auto_apms_interfaces/action/command_tree_executor.hpp"

namespace auto_apms_behavior_tree
{

class HaltExecutorAction : public RosActionNode<auto_apms_interfaces::action::CommandTreeExecutor>
{
public:
  using RosActionNode::RosActionNode;

  bool setGoal(Goal & goal)
  {
    goal.command = Goal::COMMAND_HALT;
    return true;
  }
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::HaltExecutorAction)
