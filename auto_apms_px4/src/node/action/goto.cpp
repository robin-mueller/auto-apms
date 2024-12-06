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
#include "auto_apms_interfaces/action/go_to.hpp"

#define INPUT_KEY_LATITUDE "lat"
#define INPUT_KEY_LONGITUDE "lon"
#define INPUT_KEY_ALTITUDE "alt"

namespace auto_apms_px4
{

class GoToAction : public auto_apms_behavior_tree::core::RosActionNode<auto_apms_interfaces::action::GoTo>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<double>(INPUT_KEY_LATITUDE, "Target latitude"),
       BT::InputPort<double>(INPUT_KEY_LONGITUDE, "Target longitude"),
       BT::InputPort<double>(INPUT_KEY_ALTITUDE, "Target altitude in meter (AMSL)")});
  }

  bool setGoal(Goal & goal)
  {
    goal.lat = getInput<double>(INPUT_KEY_LATITUDE).value();
    goal.lon = getInput<double>(INPUT_KEY_LONGITUDE).value();
    goal.alt = getInput<double>(INPUT_KEY_ALTITUDE).value();
    goal.head_towards_destination = true;
    return true;
  }
};

}  // namespace auto_apms_px4

AUTO_APMS_BEHAVIOR_TREE_DECLARE_NODE(auto_apms_px4::GoToAction)
