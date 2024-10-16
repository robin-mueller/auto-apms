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

#include "auto_apms_px4/constants.hpp"
#include "auto_apms_px4/mode_executor.hpp"

namespace auto_apms_px4 {

class LandTask : public ModeExecutor<auto_apms_interfaces::action::Land>
{
   public:
    explicit LandTask(const rclcpp::NodeOptions& options) : ModeExecutor{LAND_TASK_NAME, options, FlightMode::Land} {}

   private:
    bool SendActivationCommand(const VehicleCommandClient& client, std::shared_ptr<const Goal> goal_ptr)
    {
        (void)goal_ptr;
        return client.Land();
    }
};

}  // namespace auto_apms_px4

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_px4::LandTask)
