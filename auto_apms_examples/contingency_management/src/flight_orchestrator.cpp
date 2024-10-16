// Copyright 2024 Robin Müller
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

#include <auto_apms_behavior_tree/bt_executor.hpp>

using namespace auto_apms_behavior_tree;

namespace auto_apms::ops_engine {

class FlightOrchestratorExecutor : public BTExecutor
{
   public:
    FlightOrchestratorExecutor(const rclcpp::NodeOptions& options) : BTExecutor{"flight_orchestrator", options, 7777} {}

   private:
    void SetupBehaviorTreeFactory(rclcpp::Node::SharedPtr node_ptr, BT::BehaviorTreeFactory& factory) final {}
};

}  // namespace auto_apms::ops_engine

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms::ops_engine::FlightOrchestratorExecutor);
