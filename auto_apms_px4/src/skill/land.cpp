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

#include "auto_apms_interfaces/action/land.hpp"

#include "auto_apms_px4/mode_executor.hpp"
#include "px4_ros2/vehicle_state/land_detected.hpp"

namespace auto_apms_px4
{

class LandSkill : public ModeExecutor<auto_apms_interfaces::action::Land>
{
public:
  explicit LandSkill(const rclcpp::NodeOptions & options)
  : ModeExecutor(_AUTO_APMS_PX4__LAND_ACTION_NAME, options, FlightMode::Land)
  {
    px4_ros2::Context context(*node_ptr_, "");
    land_detected_ptr_ = std::make_unique<px4_ros2::LandDetected>(context);
  }

private:
  bool sendActivationCommand(
    const VehicleCommandClient & client, std::shared_ptr<const Goal> /*goal_ptr*/) override final
  {
    return client.land();
  }

  bool isCompleted(
    std::shared_ptr<const Goal> goal_ptr, const px4_msgs::msg::VehicleStatus & vehicle_status) override final
  {
    return (land_detected_ptr_->lastValid(std::chrono::seconds(3)) && land_detected_ptr_->landed()) ||
           ModeExecutor::isCompleted(goal_ptr, vehicle_status);
  }

  std::unique_ptr<px4_ros2::LandDetected> land_detected_ptr_;
};

}  // namespace auto_apms_px4

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_px4::LandSkill)
