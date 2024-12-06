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

#include "auto_apms_behavior_tree_core/node.hpp"
#include "auto_apms_interfaces/action/command_tree_executor.hpp"

#define INPUT_KEY_EXECUTOR_NAME "executor"

namespace auto_apms_behavior_tree
{

using CommandAction = auto_apms_interfaces::action::CommandTreeExecutor;

template <uint8_t command>
class CommandExecutorTemplate : public core::RosActionNode<CommandAction>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    // We do not use the default port for the action name
    return {BT::InputPort<std::string>(INPUT_KEY_EXECUTOR_NAME, "Name of the executor to command.")};
  }

  bool setGoal(Goal & goal) override final
  {
    goal.command = command;
    return true;
  }
};

class ResumeExecutor : public CommandExecutorTemplate<CommandAction::Goal::COMMAND_RESUME>
{
public:
  using CommandExecutorTemplate::CommandExecutorTemplate;
};

class PauseExecutor : public CommandExecutorTemplate<CommandAction::Goal::COMMAND_PAUSE>
{
public:
  using CommandExecutorTemplate::CommandExecutorTemplate;
};

class HaltExecutor : public CommandExecutorTemplate<CommandAction::Goal::COMMAND_HALT>
{
public:
  using CommandExecutorTemplate::CommandExecutorTemplate;
};

class TerminateExecutor : public CommandExecutorTemplate<CommandAction::Goal::COMMAND_TERMINATE>
{
public:
  using CommandExecutorTemplate::CommandExecutorTemplate;
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_DECLARE_NODE(auto_apms_behavior_tree::ResumeExecutor)
AUTO_APMS_BEHAVIOR_TREE_DECLARE_NODE(auto_apms_behavior_tree::PauseExecutor)
AUTO_APMS_BEHAVIOR_TREE_DECLARE_NODE(auto_apms_behavior_tree::HaltExecutor)
AUTO_APMS_BEHAVIOR_TREE_DECLARE_NODE(auto_apms_behavior_tree::TerminateExecutor)