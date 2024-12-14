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
#include "auto_apms_interfaces/action/arm_disarm.hpp"

#define INPUT_KEY_WAIT "wait_until_ready_to_arm"

namespace auto_apms_px4
{

class ArmAction : public auto_apms_behavior_tree::core::RosActionNode<auto_apms_interfaces::action::ArmDisarm>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<bool>(
      INPUT_KEY_WAIT, true,
      "Wait for the UAV to be ready for arming. If false and UAV is not "
      "ready to arm, will "
      "be rejected.")});
  }

  bool setGoal(Goal & goal)
  {
    goal.arming_state = Goal::ARMING_STATE_ARM;
    if (const BT::Expected<bool> expected = getInput<bool>(INPUT_KEY_WAIT)) {
      goal.wait_until_ready_to_arm = expected.value();
    } else {
      RCLCPP_ERROR(logger_, "%s", expected.error().c_str());
      return false;
    }
    return true;
  }
};

}  // namespace auto_apms_px4

AUTO_APMS_BEHAVIOR_TREE_DECLARE_NODE(auto_apms_px4::ArmAction)
