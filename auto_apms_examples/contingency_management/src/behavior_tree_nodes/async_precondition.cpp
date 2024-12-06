// Copyright 2024 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "auto_apms_behavior_tree_core/node.hpp"
#include "behaviortree_cpp/scripting/script_parser.hpp"

#define INPUT_KEY_IF "if"
#define INPUT_KEY_ELSE "else"
#define INPUT_KEY_CHECK_WHILE_RUNNING "check_while_running"

namespace auto_apms::ops_engine
{

/**
 * Same as PreconditionNode but when the condition is true, this node returns RUNNING immediately and executes the tick
 * in the next iteration.
 */
class AsyncPrecondition : public BT::DecoratorNode
{
public:
  AsyncPrecondition(const std::string & name, const BT::NodeConfig & config) : DecoratorNode(name, config)
  {
    loadExecutor();
  }

  virtual ~AsyncPrecondition() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        INPUT_KEY_IF, "If true, return RUNNING. Otherwise, return what's specified in argument '" +
                        std::string(INPUT_KEY_ELSE) + "'"),
      BT::InputPort<BT::NodeStatus>(
        INPUT_KEY_ELSE, BT::NodeStatus::SKIPPED,
        "Return status if condition is "
        "false"),
      BT::InputPort<bool>(
        INPUT_KEY_CHECK_WHILE_RUNNING, false,
        "Check the condition in argument '" + std::string(INPUT_KEY_IF) + "' also while the child is RUNNING")};
  }

private:
  virtual BT::NodeStatus tick() override final
  {
    // Load the result of the condition
    loadExecutor();
    BT::Ast::Environment env = {config().blackboard, config().enums};
    bool condition_result = _executor(env).cast<bool>();

    BT::NodeStatus else_return;
    if (!getInput(INPUT_KEY_ELSE, else_return)) {
      throw auto_apms_behavior_tree::exceptions::RosNodeError(
        "Missing parameter [" + std::string(INPUT_KEY_ELSE) + "] in Precondition");
    }

    // Return RUNNING the first time the condition is true
    if (status() == BT::NodeStatus::IDLE) {
      if (condition_result) {
        setStatus(BT::NodeStatus::RUNNING);  // Change the current status
        return BT::NodeStatus::RUNNING;
      }
      return else_return;
    }

    if (getInput<bool>(INPUT_KEY_CHECK_WHILE_RUNNING).value() && !condition_result) {
      return else_return;
    }

    // Propagate the tick
    auto const child_status = child_node_->executeTick();
    if (isStatusCompleted(child_status)) {
      resetChild();
    }
    return child_status;
  }

  void loadExecutor()
  {
    std::string script;
    if (!getInput(INPUT_KEY_IF, script)) {
      throw auto_apms_behavior_tree::exceptions::RosNodeError(
        "Missing parameter [" + std::string(INPUT_KEY_IF) + "] in Precondition");
    }
    if (script == _script) {
      return;
    }
    auto executor = BT::ParseScript(script);
    if (!executor) {
      throw auto_apms_behavior_tree::exceptions::RosNodeError(executor.error());
    } else {
      _executor = executor.value();
      _script = script;
    }
  }

  std::string _script;
  BT::ScriptFunction _executor;
};

}  // namespace auto_apms::ops_engine

AUTO_APMS_BEHAVIOR_TREE_DECLARE_NODE(auto_apms::ops_engine::AsyncPrecondition)
