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

#include <Eigen/Geometry>

#include "px4_behavior/bt_ros2_node.hpp"
#include "px4_behavior_interfaces/action/go_to.hpp"

#define INPUT_KEY_VEC "vector"

using namespace BT;

namespace px4_behavior {

class GoToVectorAction : public RosActionNode<px4_behavior_interfaces::action::GoTo>
{
   public:
    using RosActionNode::RosActionNode;

    static PortsList providedPorts()
    {
        return providedBasicPorts(
            {InputPort<Eigen::Vector3d>(INPUT_KEY_VEC, "Target position as a pointer to a vector")});
    }

    bool setGoal(Goal& goal)
    {
        if (auto any_locked = getLockedPortContent(INPUT_KEY_VEC)) {
            if (any_locked->empty()) {
                RCLCPP_ERROR(logger(), "%s - Value at blackboard entry {%s} is empty", name().c_str(), INPUT_KEY_VEC);
                return false;
            }
            else if (Eigen::Vector3d* vec_ptr = any_locked->castPtr<Eigen::Vector3d>()) {
                goal.lat = vec_ptr->x();
                goal.lon = vec_ptr->y();
                goal.alt = vec_ptr->z();
                goal.head_towards_destination = true;
                return true;
            }
            else {
                RCLCPP_ERROR(logger(), "%s - Failed to cast pointer {%s}", name().c_str(), INPUT_KEY_VEC);
                return false;
            }
        }
        RCLCPP_ERROR(logger(), "%s - getLockedPortContent() failed for argument %s", name().c_str(), INPUT_KEY_VEC);
        return false;
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
};

}  // namespace px4_behavior

#include "px4_behavior/register_behavior_tree_node_macro.hpp"
PX4_BEHAVIOR_REGISTER_BEHAVIOR_TREE_NODE(px4_behavior::GoToVectorAction);
