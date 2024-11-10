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
#include "px4_msgs/msg/vehicle_global_position.hpp"

#define OUTPUT_KEY_LAT "lat"
#define OUTPUT_KEY_LON "lon"
#define OUTPUT_KEY_ALT "alt"
#define OUTPUT_KEY_POS "pos_vec"

using GlobalPositionMsg = px4_msgs::msg::VehicleGlobalPosition;

namespace auto_apms_px4
{

class ReadGlobalPosition : public auto_apms_behavior_tree::core::RosSubscriberNode<GlobalPositionMsg>
{
  GlobalPositionMsg last_msg_;

public:
  ReadGlobalPosition(const std::string & instance_name, const Config & config, const Context & context)
  : RosSubscriberNode{instance_name, config, context, rclcpp::SensorDataQoS{}}
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::OutputPort<Eigen::Vector3d>(
         OUTPUT_KEY_POS, "{pos_vec}",
         "Current global position vector (latitude [°], "
         "longitude [°], altitude AMSL [m])"),
       BT::OutputPort<double>(OUTPUT_KEY_LAT, "{lat}", "Current latitude in degree [°]"),
       BT::OutputPort<double>(OUTPUT_KEY_LON, "{lon}", "Current longitude in degree [°]"),
       BT::OutputPort<double>(OUTPUT_KEY_ALT, "{alt}", "Current altitude in meter (AMSL)")});
  }

  BT::NodeStatus onTick(const std::shared_ptr<GlobalPositionMsg> & last_msg_ptr) final
  {
    // Check if a new message was received
    if (last_msg_ptr) {
      last_msg_ = *last_msg_ptr;
    }

    if (auto any_locked = getLockedPortContent(OUTPUT_KEY_POS)) {
      setOutput(OUTPUT_KEY_LAT, last_msg_.lat);
      setOutput(OUTPUT_KEY_LON, last_msg_.lon);
      setOutput(OUTPUT_KEY_ALT, last_msg_.alt);
      Eigen::Vector3d pos{last_msg_.lat, last_msg_.lon, last_msg_.alt};
      any_locked.assign(pos);
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_ERROR(logger_, "%s - getLockedPortContent() failed for argument %s", name().c_str(), OUTPUT_KEY_POS);
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace auto_apms_px4

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_px4::ReadGlobalPosition)
