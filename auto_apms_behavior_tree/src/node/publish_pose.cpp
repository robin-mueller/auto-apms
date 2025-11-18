// Copyright 2025 Robin Müller
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

#include "auto_apms_behavior_tree_core/node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#define INPUT_KEY_POINT_X "x"
#define INPUT_KEY_POINT_Y "y"
#define INPUT_KEY_POINT_Z "z"
#define INPUT_KEY_ORIENTATION_ROLL "roll"
#define INPUT_KEY_ORIENTATION_PITCH "pitch"
#define INPUT_KEY_ORIENTATION_YAW "yaw"
#define INPUT_KEY_USE_DEGREES "use_degrees"
#define INPUT_KEY_FRAME_ID "frame_id"

namespace auto_apms_behavior_tree
{

class PublishPose : public core::RosPublisherNode<geometry_msgs::msg::PoseStamped>
{
public:
  PublishPose(const std::string & instance_name, const Config & config, const Context & context)
  : RosPublisherNode(instance_name, config, context)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>(INPUT_KEY_POINT_X, 0.0, "Target x position in meters."),
      BT::InputPort<double>(INPUT_KEY_POINT_Y, 0.0, "Target y position in meters."),
      BT::InputPort<double>(INPUT_KEY_POINT_Z, 0.0, "Target z position in meters."),
      BT::InputPort<double>(
        INPUT_KEY_ORIENTATION_ROLL, 0.0, "Target roll angle in radians (or degrees if use_degrees is true)."),
      BT::InputPort<double>(
        INPUT_KEY_ORIENTATION_PITCH, 0.0, "Target pitch angle in radians (or degrees if use_degrees is true)."),
      BT::InputPort<double>(
        INPUT_KEY_ORIENTATION_YAW, 0.0, "Target yaw angle in radians (or degrees if use_degrees is true)."),
      BT::InputPort<bool>(
        INPUT_KEY_USE_DEGREES, false, "If true, interpret roll, pitch, yaw as degrees instead of radians."),
      BT::InputPort<std::string>(INPUT_KEY_FRAME_ID, "map", "Frame ID for the pose."),
    });
  }

  bool setMessage(geometry_msgs::msg::PoseStamped & msg) override final
  {
    // Get position inputs
    const BT::Expected<double> expected_x = getInput<double>(INPUT_KEY_POINT_X);
    const BT::Expected<double> expected_y = getInput<double>(INPUT_KEY_POINT_Y);
    const BT::Expected<double> expected_z = getInput<double>(INPUT_KEY_POINT_Z);
    const BT::Expected<double> expected_roll = getInput<double>(INPUT_KEY_ORIENTATION_ROLL);
    const BT::Expected<double> expected_pitch = getInput<double>(INPUT_KEY_ORIENTATION_PITCH);
    const BT::Expected<double> expected_yaw = getInput<double>(INPUT_KEY_ORIENTATION_YAW);
    const BT::Expected<bool> expected_use_degrees = getInput<bool>(INPUT_KEY_USE_DEGREES);
    const BT::Expected<std::string> expected_frame_id = getInput<std::string>(INPUT_KEY_FRAME_ID);

    if (
      !expected_x || !expected_y || !expected_z || !expected_roll || !expected_pitch || !expected_yaw ||
      !expected_use_degrees || !expected_frame_id) {
      RCLCPP_ERROR(logger_, "%s - Failed to get input values", context_.getFullyQualifiedTreeNodeName(this).c_str());
      return false;
    }

    msg.pose.position.x = expected_x.value();
    msg.pose.position.y = expected_y.value();
    msg.pose.position.z = expected_z.value();

    // Convert roll, pitch, yaw to quaternion using tf2
    double roll = expected_roll.value();
    double pitch = expected_pitch.value();
    double yaw = expected_yaw.value();

    // Convert degrees to radians if needed
    if (expected_use_degrees.value()) {
      constexpr double DEG_TO_RAD = M_PI / 180.0;
      roll *= DEG_TO_RAD;
      pitch *= DEG_TO_RAD;
      yaw *= DEG_TO_RAD;
    }

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    msg.pose.orientation = tf2::toMsg(q);

    // Set frame ID and timestamp
    msg.header.frame_id = expected_frame_id.value();
    msg.header.stamp = context_.getCurrentTime();

    RCLCPP_DEBUG(
      logger_, "%s - Publishing pose: [%.2f, %.2f, %.2f m] [roll=%.2f, pitch=%.2f, yaw=%.2f rad] in frame '%s'",
      context_.getFullyQualifiedTreeNodeName(this).c_str(), msg.pose.position.x, msg.pose.position.y,
      msg.pose.position.z, roll, pitch, yaw, msg.header.frame_id.c_str());

    return true;
  }
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::PublishPose)
