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
#include "pyrobosim_msgs/msg/robot_state.hpp"

#define INPUT_KEY_ROBOT "robot"
#define OUTPUT_KEY_LOCATION "location"
#define OUTPUT_KEY_BATTERY "battery"

namespace auto_apms_simulation
{

class GetRobotState : public auto_apms_behavior_tree::core::RosSubscriberNode<pyrobosim_msgs::msg::RobotState>
{
  bool first_message_received_ = false;
  MessageType last_msg_;

public:
  using RosSubscriberNode::RosSubscriberNode;

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<double>(OUTPUT_KEY_BATTERY, "{=}", "Current battery state [%]."),
      BT::OutputPort<std::string>(OUTPUT_KEY_LOCATION, "{=}", "Current location name."),
      BT::InputPort<std::string>(INPUT_KEY_ROBOT, "Name of the robot.")};
  }

  BT::NodeStatus onTick(const std::shared_ptr<MessageType> & last_msg_ptr) final
  {
    // Check if a new message was received
    if (last_msg_ptr) {
      first_message_received_ = true;
      last_msg_ = *last_msg_ptr;
    }

    if (!first_message_received_) return BT::NodeStatus::FAILURE;

    setOutput(OUTPUT_KEY_LOCATION, last_msg_.last_visited_location);
    setOutput(OUTPUT_KEY_BATTERY, last_msg_.battery_level);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace auto_apms_simulation

AUTO_APMS_BEHAVIOR_TREE_DECLARE_NODE(auto_apms_simulation::GetRobotState)
