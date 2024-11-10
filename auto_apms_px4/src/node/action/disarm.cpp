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

namespace auto_apms_px4
{

class DisarmAction : public auto_apms_behavior_tree::core::RosActionNode<auto_apms_interfaces::action::ArmDisarm>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  bool setGoal(Goal & goal)
  {
    goal.arming_state = Goal::ARMING_STATE_DISARM;
    return true;
  }
};

}  // namespace auto_apms_px4

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_px4::DisarmAction)
