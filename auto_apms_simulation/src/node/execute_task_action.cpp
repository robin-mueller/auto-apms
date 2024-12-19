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

#include "pyrobosim_msgs/action/execute_task_action.hpp"

#include "auto_apms_behavior_tree_core/node.hpp"
#include "auto_apms_simulation/util.hpp"

#define INPUT_KEY_ROBOT_NAME "robot"
#define INPUT_KEY_TARGET_LOCATION "target"
#define INPUT_KEY_ROOM "room"
#define INPUT_KEY_SOURCE_LOCATION "source"

namespace auto_apms_simulation
{

class NavigateToLocation
: public auto_apms_behavior_tree::core::RosActionNode<pyrobosim_msgs::action::ExecuteTaskAction>
{
  int32_t success_result_ = ExecutionResultMsg::SUCCESS;

public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(INPUT_KEY_SOURCE_LOCATION, "", "Name of the source location."),
      BT::InputPort<std::string>(INPUT_KEY_ROOM, "", "Name of the target room."),
      BT::InputPort<std::string>(INPUT_KEY_TARGET_LOCATION, "Name of the target location."),
      BT::InputPort<std::string>(INPUT_KEY_ROBOT_NAME, "Name of the robot.")};
  }

  bool setGoal(Goal & goal)
  {
    const BT::Expected<std::string> expected_robot = getInput<std::string>(INPUT_KEY_ROBOT_NAME);
    if (!expected_robot || expected_robot.value().empty()) {
      RCLCPP_ERROR(
        logger_, "%s - You must provide a non-empty robot name.", context_.getFullyQualifiedTreeNodeName(this).c_str());
      RCLCPP_DEBUG_EXPRESSION(
        logger_, !expected_robot, "%s - Error message: %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        expected_robot.error().c_str());
      return false;
    }
    const BT::Expected<std::string> expected_location = getInput<std::string>(INPUT_KEY_TARGET_LOCATION);
    if (!expected_location || expected_location.value().empty()) {
      RCLCPP_ERROR(
        logger_, "%s - You must provide a non-empty target location name.",
        context_.getFullyQualifiedTreeNodeName(this).c_str());
      RCLCPP_DEBUG_EXPRESSION(
        logger_, !expected_location, "%s - Error message: %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        expected_location.error().c_str());
      return false;
    }
    goal.action.type = "navigate";
    goal.action.robot = expected_robot.value();
    goal.action.target_location = expected_location.value();
    goal.action.room = getInput<std::string>(INPUT_KEY_ROOM).value();
    goal.action.source_location = getInput<std::string>(INPUT_KEY_SOURCE_LOCATION).value();
    return true;
  }

  void onHalt() override final { success_result_ = ExecutionResultMsg::CANCELED; }

  BT::NodeStatus onResultReceived(const WrappedResult & result)
  {
    const BT::NodeStatus base_status = RosActionNode::onResultReceived(result);
    if (base_status != BT::NodeStatus::SUCCESS) return base_status;
    if (result.result->execution_result.status != success_result_) {
      RCLCPP_ERROR(
        logger_, "%s - FAILURE: %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        toStr(result.result->execution_result).c_str());
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_DEBUG(
      logger_, "%s - SUCCESS: %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
      toStr(result.result->execution_result).c_str());
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace auto_apms_simulation

AUTO_APMS_BEHAVIOR_TREE_DECLARE_NODE(auto_apms_simulation::NavigateToLocation)
