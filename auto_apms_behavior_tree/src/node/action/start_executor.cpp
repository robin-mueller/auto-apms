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
#include "auto_apms_interfaces/action/start_tree_executor.hpp"

#define INPUT_KEY_TREE_BUILD_REQUEST "build_request"
#define INPUT_KEY_TREE_BUILD_HANDLER "build_handler"
#define INPUT_KEY_ROOT_TREE_NAME "root_tree"
#define INPUT_KEY_NODE_OVERRIDES "node_overrides"
#define INPUT_KEY_ATTACH "attach"
#define INPUT_KEY_CLEAR_BB "clear_blackboard"

namespace auto_apms_behavior_tree
{

class StartExecutorAction : public core::RosActionNode<auto_apms_interfaces::action::StartTreeExecutor>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<std::string>(
         INPUT_KEY_TREE_BUILD_REQUEST, "String passed to the tree build handler defining which tree is to be built."),
       BT::InputPort<std::string>(
         INPUT_KEY_TREE_BUILD_HANDLER, "",
         "Fully qualified class name of the build handler that is supposed to take care of the request."),
       BT::InputPort<std::string>(INPUT_KEY_ROOT_TREE_NAME, "", "Name of the root tree."),
       BT::InputPort<std::string>(
         INPUT_KEY_NODE_OVERRIDES, "",
         "YAML/JSON formatted string encoding the registration parameters for any tree nodes supposed to be "
         "loaded/overridden before the execution starts."),
       BT::InputPort<bool>(
         INPUT_KEY_ATTACH, true, "Boolean flag wether to attach to the execution process or start in detached mode."),
       BT::InputPort<bool>(
         INPUT_KEY_CLEAR_BB, true,
         "Boolean flag wether to clear the existing blackboard entries before the execution starts or not.")});
  }

  bool setGoal(Goal & goal) override final
  {
    goal.build_request = getInput<std::string>(INPUT_KEY_TREE_BUILD_REQUEST).value();
    goal.build_handler = getInput<std::string>(INPUT_KEY_TREE_BUILD_HANDLER).value();
    goal.root_tree = getInput<std::string>(INPUT_KEY_ROOT_TREE_NAME).value();
    goal.node_overrides = getInput<std::string>(INPUT_KEY_NODE_OVERRIDES).value();
    goal.attach = getInput<bool>(INPUT_KEY_ATTACH).value();
    goal.clear_blackboard = getInput<bool>(INPUT_KEY_CLEAR_BB).value();
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override final
  {
    RCLCPP_DEBUG(
      context_.getLogger(), "%s - Received response %i from server %s: %s", context_.getFullName(this).c_str(),
      wr.result->tree_result, getActionName().c_str(), wr.result->message.c_str());
    if (getInput<bool>(INPUT_KEY_ATTACH).value()) {
      if (wr.result->tree_result == ActionType::Result::TREE_RESULT_SUCCESS) return BT::NodeStatus::SUCCESS;
      return BT::NodeStatus::FAILURE;
    }
    if (wr.result->tree_result == ActionType::Result::TREE_RESULT_NOT_SET) return BT::NodeStatus::SUCCESS;
    throw exceptions::RosNodeError(
      "Expected tree_result to be TREE_RESULT_NOT_SET when requesting to start in detached mode.");
  }
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::StartExecutorAction)
