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

#include "px4_behavior_interfaces/action/takeoff.hpp"

#include "px4_behavior/px4_behavior.hpp"

namespace px4_behavior {

class TakeoffTask : public ModeExecutor<px4_behavior_interfaces::action::Takeoff>
{
   public:
    explicit TakeoffTask(const rclcpp::NodeOptions& options)
        : ModeExecutor{TAKEOFF_TASK_NAME, options, FlightMode::Takeoff}
    {}

   private:
    bool SendActivationCommand(const VehicleCommandClient& client, std::shared_ptr<const Goal> goal_ptr)
    {
        return client.Takeoff(goal_ptr->altitude_amsl_m, goal_ptr->heading_rad);
    }
};

}  // namespace px4_behavior

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(px4_behavior::TakeoffTask)
