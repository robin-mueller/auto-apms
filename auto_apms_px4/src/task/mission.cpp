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

#include "auto_apms_interfaces/action/mission.hpp"

#include "auto_apms_px4/constants.hpp"
#include "auto_apms_px4/mode_executor.hpp"

namespace auto_apms_px4
{

class MissionTask : public ModeExecutor<auto_apms_interfaces::action::Mission>
{
public:
  explicit MissionTask(const rclcpp::NodeOptions& options)
    : ModeExecutor{ MISSION_TASK_NAME, options, FlightMode::Mission }
  {
  }

private:
  bool SendActivationCommand(const VehicleCommandClient& client, std::shared_ptr<const Goal> goal_ptr) final
  {
    if (goal_ptr->do_restart)
    {
      return client.StartMission();
    }
    return client.SyncActivateFlightMode(FlightMode::Mission);
  }
};

}  // namespace auto_apms_px4

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_px4::MissionTask)
