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

#include "auto_apms_behavior_tree/node_plugin.hpp"
#include "auto_apms_px4/bt_px4_topic_sub_node.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"

#define OUTPUT_KEY_LAT "lat"
#define OUTPUT_KEY_LON "lon"
#define OUTPUT_KEY_ALT "alt"
#define OUTPUT_KEY_POS "pos_vec"

using namespace BT;
using GlobalPositionMsg = px4_msgs::msg::VehicleGlobalPosition;

namespace auto_apms_px4 {

class ReadGlobalPosition : public PX4RosTopicSubNode<GlobalPositionMsg>
{
    GlobalPositionMsg last_msg_;

   public:
    using PX4RosTopicSubNode::PX4RosTopicSubNode;

    static PortsList providedPorts()
    {
        return providedBasicPorts(
            {OutputPort<Eigen::Vector3d>(
                 OUTPUT_KEY_POS,
                 "{pos_vec}",
                 "Current global position vector (latitude [°], longitude [°], altitude AMSL [m])"),
             OutputPort<double>(OUTPUT_KEY_LAT, "{lat}", "Current latitude in degree [°]"),
             OutputPort<double>(OUTPUT_KEY_LON, "{lon}", "Current longitude in degree [°]"),
             OutputPort<double>(OUTPUT_KEY_ALT, "{alt}", "Current altitude in meter (AMSL)")});
    }

    NodeStatus onTick(const std::shared_ptr<GlobalPositionMsg>& last_msg_ptr) final
    {
        // Check if a new message was received
        if (last_msg_ptr) { last_msg_ = *last_msg_ptr; }

        if (auto any_locked = getLockedPortContent(OUTPUT_KEY_POS)) {
            setOutput(OUTPUT_KEY_LAT, last_msg_.lat);
            setOutput(OUTPUT_KEY_LON, last_msg_.lon);
            setOutput(OUTPUT_KEY_ALT, last_msg_.alt);
            Eigen::Vector3d pos{last_msg_.lat, last_msg_.lon, last_msg_.alt};
            any_locked.assign(pos);
            return NodeStatus::SUCCESS;
        }
        RCLCPP_ERROR(logger(), "%s - getLockedPortContent() failed for argument %s", name().c_str(), OUTPUT_KEY_POS);
        return NodeStatus::FAILURE;
    }
};

}  // namespace auto_apms_px4

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_px4::ReadGlobalPosition)
