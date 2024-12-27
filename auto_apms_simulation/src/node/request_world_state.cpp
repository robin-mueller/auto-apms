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

#include "pyrobosim_msgs/srv/request_world_state.hpp"

#include <regex>

#include "auto_apms_behavior_tree_core/node.hpp"
#include "auto_apms_simulation/util.hpp"
#include "pyrobosim_msgs/msg/robot_state.hpp"

#define INPUT_KEY_TARGET_LOCATION "target_loc"
#define INPUT_KEY_FILTER_LOCATION "filter_loc"
#define INPUT_KEY_FILTER_ROBOT "filter_robot"

namespace auto_apms_simulation
{

class IsLocationOccupied : public auto_apms_behavior_tree::core::RosServiceNode<pyrobosim_msgs::srv::RequestWorldState>
{
  std::string target_location_;

public:
  using RosServiceNode::RosServiceNode;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(INPUT_KEY_FILTER_ROBOT, ".*", "Regex filter for robots to consider."),
      BT::InputPort<std::string>(
        INPUT_KEY_FILTER_LOCATION, ".*", "Regex filter for locations to match the target location."),
      BT::InputPort<std::string>(INPUT_KEY_TARGET_LOCATION, "Name of the location to test for occupancy.")};
  }

  bool setRequest(Request::SharedPtr & request)
  {
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
    target_location_ = expected_location.value();
    request->robot = "";  // Request state for all robots
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    for (const pyrobosim_msgs::msg::RobotState & robot_state : response->state.robots) {
      std::regex loc_regex(getInput<std::string>(INPUT_KEY_FILTER_LOCATION).value());
      std::regex robot_regex(getInput<std::string>(INPUT_KEY_FILTER_ROBOT).value());
      if (
        std::regex_match(robot_state.last_visited_location, loc_regex) &&
        std::regex_match(robot_state.name, robot_regex) && robot_state.last_visited_location == target_location_)
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace auto_apms_simulation

AUTO_APMS_BEHAVIOR_TREE_DECLARE_NODE(auto_apms_simulation::IsLocationOccupied)