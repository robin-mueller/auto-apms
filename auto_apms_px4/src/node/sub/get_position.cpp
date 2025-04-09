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
#include <type_traits>

#include "auto_apms_behavior_tree_core/node.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

#define OUTPUT_KEY_LAT "lat"
#define OUTPUT_KEY_LON "lon"
#define OUTPUT_KEY_ALT "alt"
#define OUTPUT_KEY_N "north"
#define OUTPUT_KEY_E "east"
#define OUTPUT_KEY_D "down"
#define OUTPUT_KEY_VEC "vector"

using GlobalPositionMsg = px4_msgs::msg::VehicleGlobalPosition;
using LocalPositionMsg = px4_msgs::msg::VehicleLocalPosition;

namespace auto_apms_px4
{

template <class T>
class GetPosition : public auto_apms_behavior_tree::core::RosSubscriberNode<T>
{
  bool first_message_received_ = false;
  T last_msg_;

public:
  GetPosition(
    const std::string & instance_name, const BT::NodeConfig & config,
    const auto_apms_behavior_tree::core::RosNodeContext & context)
  : auto_apms_behavior_tree::core::RosSubscriberNode<T>{instance_name, config, context, rclcpp::SensorDataQoS{}}
  {
  }

  static BT::PortsList providedPorts()
  {
    if constexpr (std::is_same_v<T, GlobalPositionMsg>) {
      return {
        BT::OutputPort<Eigen::MatrixXd>(
          OUTPUT_KEY_VEC, "{pos_vec}",
          "Current global position vector (latitude [°], longitude [°], altitude AMSL [m])"),
        BT::OutputPort<double>(OUTPUT_KEY_LAT, "{lat}", "Current latitude in degree [°]"),
        BT::OutputPort<double>(OUTPUT_KEY_LON, "{lon}", "Current longitude in degree [°]"),
        BT::OutputPort<double>(OUTPUT_KEY_ALT, "{alt}", "Current altitude in meter (AMSL)")};
    } else {
      return {
        BT::OutputPort<Eigen::MatrixXd>(
          OUTPUT_KEY_VEC, "{pos_vec}", "Current local position vector (north [m], east [m], down [m])"),
        BT::OutputPort<double>(OUTPUT_KEY_N, "{north}", "Current north [m] relative to origin"),
        BT::OutputPort<double>(OUTPUT_KEY_E, "{east}", "Current east [m] relative to origin"),
        BT::OutputPort<double>(OUTPUT_KEY_D, "{down}", "Current down [m] relative to origin")};
    }
  }

  BT::NodeStatus onTick(const std::shared_ptr<T> & last_msg_ptr) final
  {
    // Check if a new message was received
    if (last_msg_ptr) {
      first_message_received_ = true;
      last_msg_ = *last_msg_ptr;
    }

    if (!first_message_received_) return BT::NodeStatus::FAILURE;

    if constexpr (std::is_same_v<T, GlobalPositionMsg>) {
      Eigen::MatrixXd mat(1, 3);
      mat(0, 0) = last_msg_.lat;
      mat(0, 1) = last_msg_.lon;
      mat(0, 2) = last_msg_.alt;
      this->setOutput(OUTPUT_KEY_LAT, mat(0, 0));
      this->setOutput(OUTPUT_KEY_LON, mat(0, 1));
      this->setOutput(OUTPUT_KEY_ALT, mat(0, 2));
      this->setOutput(OUTPUT_KEY_VEC, mat);
    } else {
      Eigen::MatrixXd mat(1, 3);
      mat(0, 0) = last_msg_.x;
      mat(0, 1) = last_msg_.y;
      mat(0, 2) = last_msg_.z;
      this->setOutput(OUTPUT_KEY_N, mat(0, 0));
      this->setOutput(OUTPUT_KEY_E, mat(0, 1));
      this->setOutput(OUTPUT_KEY_D, mat(0, 2));
      this->setOutput(OUTPUT_KEY_VEC, mat);
    }

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace auto_apms_px4

AUTO_APMS_BEHAVIOR_TREE_DECLARE_NODE(auto_apms_px4::GetPosition<GlobalPositionMsg>)
AUTO_APMS_BEHAVIOR_TREE_DECLARE_NODE(auto_apms_px4::GetPosition<LocalPositionMsg>)
