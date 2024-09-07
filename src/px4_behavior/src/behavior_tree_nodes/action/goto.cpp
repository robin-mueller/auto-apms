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

#include "px4_behavior/bt_ros2_node.hpp"
#include "px4_behavior_interfaces/action/go_to.hpp"

#define INPUT_KEY_LATITUDE "lat"
#define INPUT_KEY_LONGITUDE "lon"
#define INPUT_KEY_ALTITUDE "alt"

using namespace BT;

namespace px4_behavior {

class GoToAction : public RosActionNode<px4_behavior_interfaces::action::GoTo>
{
   public:
    using RosActionNode::RosActionNode;

    static PortsList providedPorts()
    {
        return providedBasicPorts({InputPort<double>(INPUT_KEY_LATITUDE, "Target latitude"),
                                   InputPort<double>(INPUT_KEY_LONGITUDE, "Target longitude"),
                                   InputPort<double>(INPUT_KEY_ALTITUDE, "Target altitude in meter (AMSL)")});
    }

    bool setGoal(Goal& goal)
    {
        goal.lat = getInput<double>(INPUT_KEY_LATITUDE).value();
        goal.lon = getInput<double>(INPUT_KEY_LONGITUDE).value();
        goal.alt = getInput<double>(INPUT_KEY_ALTITUDE).value();
        goal.head_towards_destination = true;
        return true;
    }

    NodeStatus onResultReceived(const WrappedResult& wr)
    {
        if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) return NodeStatus::SUCCESS;
        return NodeStatus::FAILURE;
    }

    NodeStatus onFailure(ActionNodeErrorCode error)
    {
        RCLCPP_ERROR(logger(), "%s - Error: %d - %s", name().c_str(), error, toStr(error));
        return NodeStatus::FAILURE;
    }
};

}  // namespace px4_behavior

#include "px4_behavior/register_behavior_tree_node_macro.hpp"
PX4_BEHAVIOR_REGISTER_BEHAVIOR_TREE_NODE(px4_behavior::GoToAction);
