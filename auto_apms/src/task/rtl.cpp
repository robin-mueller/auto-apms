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

#include "auto_apms_interfaces/action/rtl.hpp"

#include "auto_apms/constants.hpp"
#include "auto_apms/mode_executor.hpp"

namespace auto_apms {

class RTLTask : public ModeExecutor<auto_apms_interfaces::action::RTL>
{
   public:
    explicit RTLTask(const rclcpp::NodeOptions& options) : ModeExecutor{RTL_TASK_NAME, options, FlightMode::RTL} {}

   private:
    // PX4 seems to not always give a completed signal for RTL, so check for disarmed as a fallback completed state
    bool IsCompleted(std::shared_ptr<const Goal> goal_ptr, const px4_msgs::msg::VehicleStatus& vehicle_status)
    {
        return ModeExecutor::IsCompleted(goal_ptr, vehicle_status) ||
               vehicle_status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
    }
};

}  // namespace auto_apms

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms::RTLTask)
