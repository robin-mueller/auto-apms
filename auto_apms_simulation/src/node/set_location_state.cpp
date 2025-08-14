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

#include "pyrobosim_msgs/srv/set_location_state.hpp"

#include "auto_apms_behavior_tree_core/node.hpp"
#include "auto_apms_simulation/util.hpp"

#define INPUT_KEY_LOCATION "location"
#define INPUT_KEY_OPEN "open"
#define INPUT_KEY_LOCK "lock"

namespace auto_apms_simulation
{

class SetLocationState : public auto_apms_behavior_tree::core::RosServiceNode<pyrobosim_msgs::srv::SetLocationState>
{
public:
  using RosServiceNode::RosServiceNode;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>(INPUT_KEY_LOCK, false, "Lock/Unlock the location."),
      BT::InputPort<bool>(INPUT_KEY_OPEN, true, "Open/Close the location."),
      BT::InputPort<std::string>(INPUT_KEY_LOCATION, "Name of the location.")};
  }

  bool setRequest(Request::SharedPtr & request) override final
  {
    const BT::Expected<std::string> expected_location = getInput<std::string>(INPUT_KEY_LOCATION);
    if (!expected_location || expected_location.value().empty()) {
      RCLCPP_ERROR(
        logger_, "%s - You must provide a non-empty location name.",
        context_.getFullyQualifiedTreeNodeName(this).c_str());
      RCLCPP_DEBUG_EXPRESSION(
        logger_, !expected_location, "%s - Error message: %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        expected_location.error().c_str());
      return false;
    }
    request->location_name = expected_location.value();
    request->open = getInput<bool>(INPUT_KEY_OPEN).value();
    request->lock = getInput<bool>(INPUT_KEY_LOCK).value();
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    const BT::NodeStatus base_status = RosServiceNode::onResponseReceived(response);
    if (base_status != BT::NodeStatus::SUCCESS) return base_status;
    if (response->result.status != ExecutionResultMsg::SUCCESS) {
      RCLCPP_ERROR(
        logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), toStr(response->result).c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace auto_apms_simulation

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_simulation::SetLocationState)