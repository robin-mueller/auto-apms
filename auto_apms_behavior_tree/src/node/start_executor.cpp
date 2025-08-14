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

#define INPUT_KEY_EXECUTOR_NAME "executor"
#define INPUT_KEY_TREE_BUILD_REQUEST "build_request"
#define INPUT_KEY_TREE_BUILD_HANDLER "build_handler"
#define INPUT_KEY_ENTRYPOINT "entrypoint"
#define INPUT_KEY_NODE_MANIFEST "node_manifest"
#define INPUT_KEY_ATTACH "attach"
#define INPUT_KEY_CLEAR_BB "clear_blackboard"

namespace auto_apms_behavior_tree
{

class StartExecutor : public core::RosActionNode<auto_apms_interfaces::action::StartTreeExecutor>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    // We do not use the default port for the action name
    return {
      BT::InputPort<bool>(
        INPUT_KEY_ATTACH, true, "Boolean flag wether to attach to the execution process or start in detached mode."),
      BT::InputPort<bool>(
        INPUT_KEY_CLEAR_BB, true,
        "Boolean flag wether to clear the existing blackboard entries before the execution starts or not."),
      BT::InputPort<std::string>(
        INPUT_KEY_NODE_MANIFEST, "",
        "YAML/JSON formatted string encoding the name and the registration options for the tree nodes supposed to be "
        "loaded before building the tree."),
      BT::InputPort<std::string>(
        INPUT_KEY_ENTRYPOINT, "",
        "Entrypoint for the behavior. If empty, let the build handler determine the entrypoint."),
      BT::InputPort<std::string>(
        INPUT_KEY_TREE_BUILD_HANDLER, "",
        "Fully qualified class name of the build handler that is supposed to take care of the request. If empty, use "
        "the current one."),
      BT::InputPort<std::string>(
        INPUT_KEY_TREE_BUILD_REQUEST, "String passed to the tree build handler defining which tree is to be built."),
      BT::InputPort<std::string>(
        INPUT_KEY_EXECUTOR_NAME,
        "Name of the executor responsible for building and running the specified behavior tree."),
    };
  }

  bool setGoal(Goal & goal) override final
  {
    const BT::Expected<std::string> expected_build_request = getInput<std::string>(INPUT_KEY_TREE_BUILD_REQUEST);
    if (!expected_build_request || expected_build_request.value().empty()) {
      RCLCPP_ERROR(
        logger_, "%s - You must provide a non-empty build request.",
        context_.getFullyQualifiedTreeNodeName(this).c_str());
      RCLCPP_DEBUG_EXPRESSION(
        logger_, !expected_build_request, "%s - Error message: %s",
        context_.getFullyQualifiedTreeNodeName(this).c_str(), expected_build_request.error().c_str());
      return false;
    }
    goal.build_request = expected_build_request.value();
    if (const BT::Expected<std::string> expected = getInput<std::string>(INPUT_KEY_TREE_BUILD_HANDLER)) {
      goal.build_handler = expected.value();
    } else {
      RCLCPP_ERROR(logger_, "%s", expected.error().c_str());
      return false;
    }
    if (const BT::Expected<std::string> expected = getInput<std::string>(INPUT_KEY_ENTRYPOINT)) {
      goal.entrypoint = expected.value();
    } else {
      RCLCPP_ERROR(logger_, "%s", expected.error().c_str());
      return false;
    }
    if (const BT::Expected<std::string> expected = getInput<std::string>(INPUT_KEY_NODE_MANIFEST)) {
      goal.node_manifest = expected.value();
    } else {
      RCLCPP_ERROR(logger_, "%s", expected.error().c_str());
      return false;
    }
    if (const BT::Expected<bool> expected = getInput<bool>(INPUT_KEY_ATTACH)) {
      goal.attach = expected.value();
    } else {
      RCLCPP_ERROR(logger_, "%s", expected.error().c_str());
      return false;
    }
    if (const BT::Expected<bool> expected = getInput<bool>(INPUT_KEY_CLEAR_BB)) {
      goal.clear_blackboard = expected.value();
    } else {
      RCLCPP_ERROR(logger_, "%s", expected.error().c_str());
      return false;
    }
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override final
  {
    switch (wr.code) {
      case rclcpp_action::ResultCode::ABORTED: {
        const std::string msg =
          context_.getFullyQualifiedTreeNodeName(this) + " - Received result ABORTED: " + wr.result->message;
        RCLCPP_ERROR_STREAM(logger_, msg);
        throw exceptions::RosNodeError(msg);
        break;
      }
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_DEBUG(
          logger_, "%s - Received result CANCELED: %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
          wr.result->message.c_str());
        // Return value is arbitrary because it is ignored when canceled
        return BT::NodeStatus::SUCCESS;
      default:
        break;
    }

    /*  If action succeeded */

    RCLCPP_DEBUG(
      logger_, "%s - Tree execution finished successfully with result %i: %s",
      context_.getFullyQualifiedTreeNodeName(this).c_str(), wr.result->tree_result, wr.result->message.c_str());

    // If started in attached mode
    if (getInput<bool>(INPUT_KEY_ATTACH).value()) {
      if (wr.result->tree_result == ActionType::Result::TREE_RESULT_SUCCESS) return BT::NodeStatus::SUCCESS;
      return BT::NodeStatus::FAILURE;
    }

    // If started in detached mode
    if (wr.result->tree_result == ActionType::Result::TREE_RESULT_NOT_SET) return BT::NodeStatus::SUCCESS;
    const std::string msg = context_.getFullyQualifiedTreeNodeName(this) +
                            " - Expected tree_result to be TREE_RESULT_NOT_SET when started in detached mode.";
    RCLCPP_ERROR_STREAM(logger_, msg);
    throw exceptions::RosNodeError(msg);
  }
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::StartExecutor)
