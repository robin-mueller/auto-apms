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

#include "auto_apms_interfaces/action/takeoff.hpp"

#include "auto_apms_behavior_tree_core/node.hpp"

#define INPUT_KEY_ALTITUDE "alt"

namespace auto_apms_px4
{

class TakeoffAction : public auto_apms_behavior_tree::core::RosActionNode<auto_apms_interfaces::action::Takeoff>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<double>(INPUT_KEY_ALTITUDE, "Target takeoff altitude in meter (AMSL)")});
  }

  bool setGoal(Goal & goal)
  {
    if (const BT::Expected<double> expected = getInput<double>(INPUT_KEY_ALTITUDE)) {
      goal.altitude_amsl_m = expected.value();
    } else {
      RCLCPP_ERROR(logger_, "%s", expected.error().c_str());
      return false;
    }
    return true;
  }
};

}  // namespace auto_apms_px4

AUTO_APMS_BEHAVIOR_TREE_DECLARE_NODE(auto_apms_px4::TakeoffAction)
