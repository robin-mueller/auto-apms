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

#define INPUT_KEY_FRAME "frame"
#define INPUT_KEY_VEC "vector"
#define INPUT_KEY_X "x"
#define INPUT_KEY_Y "y"
#define INPUT_KEY_Z "z"
#define INPUT_KEY_YAW "yaw"
#define INPUT_KEY_MAX_HOR_VEL "max_horizontal_vel"
#define INPUT_KEY_MAX_VER_VEL "max_vertical_vel"
#define INPUT_KEY_MAX_HEADING_RATE "max_heading_rate"
#define INPUT_KEY_REACHED_THRESH_POS "reached_thresh_pos"
#define INPUT_KEY_REACHED_THRESH_VEL "reached_thresh_vel"
#define INPUT_KEY_REACHED_THRESH_YAW "reached_thresh_yaw"

namespace auto_apms_px4
{

class GoToAction : public auto_apms_behavior_tree::core::RosActionNode<auto_apms_interfaces::action::GoTo>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        INPUT_KEY_FRAME, "global",
        "Reference frame: 'global' (Latitude, longitude, altitude (AMSL)) or 'local' (North, east, down from start)"),
      BT::InputPort<Eigen::MatrixXd>(INPUT_KEY_VEC, "Target position as a row vector (separated by ;)"),
      BT::InputPort<double>(INPUT_KEY_X, "Override vector entry X"),
      BT::InputPort<double>(INPUT_KEY_Y, "Override vector entry Y"),
      BT::InputPort<double>(INPUT_KEY_Z, "Override vector entry Z"),
      BT::InputPort<double>(INPUT_KEY_YAW, "Desired yaw position in degree from north (heading) [-180°, 180)"),
      BT::InputPort<double>(INPUT_KEY_MAX_HOR_VEL, 10.0, "Maximum horizontal velocity [m/s]"),
      BT::InputPort<double>(INPUT_KEY_MAX_VER_VEL, 5.0, "Maximum vertical velocity [m/s]"),
      BT::InputPort<double>(INPUT_KEY_MAX_HEADING_RATE, 30.0, "Maximum heading rate [°/s]"),
      BT::InputPort<double>(
        INPUT_KEY_REACHED_THRESH_POS, .5, "Maximum position error [m] under which the position is considered reached"),
      BT::InputPort<double>(
        INPUT_KEY_REACHED_THRESH_VEL, .3,
        "Maximum velocity error [m/s] under which the position is considered reached"),
      BT::InputPort<double>(
        INPUT_KEY_REACHED_THRESH_YAW, 7.0, "Maximum heading error [°] under which the position is considered reached")};
  }

  bool setGoal(Goal & goal)
  {
    // Node selects the correct topic under the hood using this field
    if (const BT::Expected<std::string> expected = getInput<std::string>(INPUT_KEY_FRAME)) {
      const std::string frame_str = expected.value();
      if (frame_str != "global" && frame_str != "local") {
        RCLCPP_ERROR(
          logger_, "%s - Invalid reference frame '%s'", context_.getFullyQualifiedTreeNodeName(this).c_str(),
          frame_str.c_str());
        return false;
      }
    } else {
      RCLCPP_ERROR(logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected.error().c_str());
      return false;
    }

    int position_valid = 0;
    BT::PortsRemapping::iterator input_port_it = config().input_ports.find(INPUT_KEY_VEC);
    if (input_port_it != config().input_ports.end() && !input_port_it->second.empty()) {
      if (const BT::Expected<Eigen::MatrixXd> expected = getInput<Eigen::MatrixXd>(INPUT_KEY_VEC)) {
        const Eigen::MatrixXd & mat = expected.value();
        if (mat.rows() == 1 && mat.cols() == 3) {
          goal.pos.x = mat(0, 0);
          goal.pos.y = mat(0, 1);
          goal.pos.z = mat(0, 2);
          position_valid = 3;
        } else {
          RCLCPP_ERROR(
            logger_, "%s - Input vector has wrong size (Required: 3 - Actual: %i)",
            context_.getFullyQualifiedTreeNodeName(this).c_str(), static_cast<int>(mat.rows()));
          return false;
        }
      } else {
        RCLCPP_ERROR(
          logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected.error().c_str());
        return false;
      }
    }

    input_port_it = config().input_ports.find(INPUT_KEY_X);
    if (input_port_it != config().input_ports.end() && !input_port_it->second.empty()) {
      if (const BT::Expected<double> expected = getInput<double>(INPUT_KEY_X)) {
        goal.pos.x = expected.value();
        position_valid++;
      } else {
        RCLCPP_ERROR(
          logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected.error().c_str());
        return false;
      }
    }

    input_port_it = config().input_ports.find(INPUT_KEY_Y);
    if (input_port_it != config().input_ports.end() && !input_port_it->second.empty()) {
      if (const BT::Expected<double> expected = getInput<double>(INPUT_KEY_Y)) {
        goal.pos.y = expected.value();
        position_valid++;
      } else {
        RCLCPP_ERROR(
          logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected.error().c_str());
        return false;
      }
    }

    input_port_it = config().input_ports.find(INPUT_KEY_Z);
    if (input_port_it != config().input_ports.end() && !input_port_it->second.empty()) {
      if (const BT::Expected<double> expected = getInput<double>(INPUT_KEY_Z)) {
        goal.pos.z = expected.value();
        position_valid++;
      } else {
        RCLCPP_ERROR(
          logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected.error().c_str());
        return false;
      }
    }

    if (position_valid < 3) {
      RCLCPP_ERROR(
        logger_, "%s - Position is not valid. You must provide information for all three vector entries (%i missing)",
        context_.getFullyQualifiedTreeNodeName(this).c_str(), static_cast<int>(3 - position_valid));
      return false;
    }

    input_port_it = config().input_ports.find(INPUT_KEY_YAW);
    if (input_port_it == config().input_ports.end() || input_port_it->second.empty()) {
      goal.head_towards_destination = true;
    } else {
      if (const BT::Expected<double> expected = getInput<double>(INPUT_KEY_YAW)) {
        goal.head_towards_destination = false;
        goal.heading_clockwise_from_north_deg = expected.value();
      } else {
        RCLCPP_ERROR(
          logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected.error().c_str());
        return false;
      }
    }

    if (const BT::Expected<double> expected = getInput<double>(INPUT_KEY_MAX_HOR_VEL)) {
      goal.max_horizontal_vel_m_s = expected.value();
    } else {
      RCLCPP_ERROR(logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected.error().c_str());
      return false;
    }

    if (const BT::Expected<double> expected = getInput<double>(INPUT_KEY_MAX_VER_VEL)) {
      goal.max_vertical_vel_m_s = expected.value();
    } else {
      RCLCPP_ERROR(logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected.error().c_str());
      return false;
    }

    if (const BT::Expected<double> expected = getInput<double>(INPUT_KEY_MAX_HEADING_RATE)) {
      goal.max_heading_rate_deg_s = expected.value();
    } else {
      RCLCPP_ERROR(logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected.error().c_str());
      return false;
    }

    if (const BT::Expected<double> expected = getInput<double>(INPUT_KEY_REACHED_THRESH_POS)) {
      goal.reached_thresh_pos_m = expected.value();
    } else {
      RCLCPP_ERROR(logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected.error().c_str());
      return false;
    }

    if (const BT::Expected<double> expected = getInput<double>(INPUT_KEY_REACHED_THRESH_VEL)) {
      goal.reached_thresh_vel_m_s = expected.value();
    } else {
      RCLCPP_ERROR(logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected.error().c_str());
      return false;
    }

    if (const BT::Expected<double> expected = getInput<double>(INPUT_KEY_REACHED_THRESH_YAW)) {
      goal.reached_thresh_heading_deg = expected.value();
    } else {
      RCLCPP_ERROR(logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected.error().c_str());
      return false;
    }

    return true;
  }
};

}  // namespace auto_apms_px4

AUTO_APMS_BEHAVIOR_TREE_DECLARE_NODE(auto_apms_px4::GoToAction)
