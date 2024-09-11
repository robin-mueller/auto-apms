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

#include "auto_apms/bt_ros2_node.hpp"

#define INPUT_KEY_ALTITUDE "alt"

using namespace BT;

namespace auto_apms {

class TakeoffAction : public RosActionNode<auto_apms_interfaces::action::Takeoff>
{
   public:
    using RosActionNode::RosActionNode;

    static PortsList providedPorts()
    {
        return providedBasicPorts({InputPort<double>(INPUT_KEY_ALTITUDE, "Target takeoff altitude in meter (AMSL)")});
    }

    bool setGoal(Goal& goal)
    {
        goal.altitude_amsl_m = getInput<double>(INPUT_KEY_ALTITUDE).value();
        return true;
    }

    NodeStatus onResultReceived(const WrappedResult& wr)
    {
        if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) { return NodeStatus::SUCCESS; }
        return NodeStatus::FAILURE;
    }

    NodeStatus onFailure(ActionNodeErrorCode error)
    {
        RCLCPP_ERROR(logger(), "%s - Error: %d - %s", name().c_str(), error, toStr(error));
        return NodeStatus::FAILURE;
    }

    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
    {
        (void)feedback;
        return NodeStatus::RUNNING;
    }
};

}  // namespace auto_apms

#include "auto_apms/register_behavior_tree_node_macro.hpp"
AUTO_APMS_REGISTER_BEHAVIOR_TREE_NODE(auto_apms::TakeoffAction);
