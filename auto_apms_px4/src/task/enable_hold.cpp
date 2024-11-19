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

#include "auto_apms_interfaces/action/enable_hold.hpp"

#include "auto_apms_px4/mode_executor.hpp"

namespace auto_apms_px4
{

class EnableHoldTask : public ModeExecutor<auto_apms_interfaces::action::EnableHold>
{
public:
  explicit EnableHoldTask(const rclcpp::NodeOptions & options)
  : ModeExecutor{_AUTO_APMS_PX4__ENABLE_HOLD_ACTION_NAME, options, FlightMode::Hold, false}
  {
  }

private:
  bool isCompleted(std::shared_ptr<const Goal> goal_ptr, const px4_msgs::msg::VehicleStatus & vehicle_status) final
  {
    (void)goal_ptr;
    (void)vehicle_status;
    // For HOLD mode there is no completion signal. Once activated it is considered complete.
    return true;
  }
};

}  // namespace auto_apms_px4

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_px4::EnableHoldTask)
