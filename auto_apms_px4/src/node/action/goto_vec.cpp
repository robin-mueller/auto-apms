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

#include "auto_apms_behavior_tree_core/node.hpp"
#include "auto_apms_interfaces/action/go_to.hpp"

#define INPUT_KEY_VEC "vector"

namespace auto_apms_px4
{

class GoToVectorAction : public auto_apms_behavior_tree::core::RosActionNode<auto_apms_interfaces::action::GoTo>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<Eigen::Vector3d>(INPUT_KEY_VEC, "Target position as a pointer to a vector")});
  }

  bool setGoal(Goal & goal)
  {
    if (auto any_locked = getLockedPortContent(INPUT_KEY_VEC)) {
      if (any_locked->empty()) {
        RCLCPP_ERROR(
          context_.getLogger(), "%s - Value at blackboard entry {%s} is empty",
          context_.getFullyQualifiedTreeNodeName(this).c_str(), INPUT_KEY_VEC);
        return false;
      } else if (Eigen::Vector3d * vec_ptr = any_locked->castPtr<Eigen::Vector3d>()) {
        goal.lat = vec_ptr->x();
        goal.lon = vec_ptr->y();
        goal.alt = vec_ptr->z();
        goal.head_towards_destination = true;
        return true;
      } else {
        RCLCPP_ERROR(
          context_.getLogger(), "%s - Failed to cast pointer {%s}",
          context_.getFullyQualifiedTreeNodeName(this).c_str(), INPUT_KEY_VEC);
        return false;
      }
    }
    RCLCPP_ERROR(
      context_.getLogger(), "%s - getLockedPortContent() failed for argument %s",
      context_.getFullyQualifiedTreeNodeName(this).c_str(), INPUT_KEY_VEC);
    return false;
  }
};

}  // namespace auto_apms_px4

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_px4::GoToVectorAction)
