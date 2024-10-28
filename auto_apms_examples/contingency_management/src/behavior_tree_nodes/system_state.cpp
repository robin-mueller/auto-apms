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

#include <auto_apms_examples/msg/system_state.hpp>

#include "auto_apms_behavior_tree/node/plugin.hpp"

#define OUTPUT_KEY_BATTERY "battery_level"

using SystemStateMsg = auto_apms_examples::msg::SystemState;

namespace auto_apms::ops_engine
{

class GetSystemState : public auto_apms_behavior_tree::RosSubscriberNode<SystemStateMsg>
{
  SystemStateMsg last_msg_;

public:
  using RosSubscriberNode::RosSubscriberNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
        { BT::OutputPort<float>(OUTPUT_KEY_BATTERY, "{battery_level}", "Battery level in percent") });
  }

  BT::NodeStatus onTick(const std::shared_ptr<SystemStateMsg>& last_msg_ptr) override final
  {
    // Check if a new message was received
    if (last_msg_ptr)
    {
      last_msg_ = *last_msg_ptr;
    }

    setOutput(OUTPUT_KEY_BATTERY, last_msg_.battery_level_percent);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace auto_apms::ops_engine

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms::ops_engine::GetSystemState)
