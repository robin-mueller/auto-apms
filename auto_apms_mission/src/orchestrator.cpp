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

#include "auto_apms_behavior_tree/executor/executor_server.hpp"
#include "auto_apms_behavior_tree/tree.hpp"

inline const std::string BASE_TREE_RESOURCE_ID = "orchestrator_base::MissionOrchestrator::auto_apms_mission";

namespace auto_apms_mission
{

class MissionOrchestrator : public auto_apms_behavior_tree::TreeExecutorServer
{
public:
  explicit MissionOrchestrator(rclcpp::NodeOptions options);

private:
  void setUpBuilder(TreeBuilder& builder) override final;
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

MissionOrchestrator::MissionOrchestrator(rclcpp::NodeOptions options)
: TreeExecutorServer("mission_orchestrator", options)
{
}

void MissionOrchestrator::setUpBuilder(TreeBuilder & builder) {
  builder.mergeTreesFromResource(auto_apms_behavior_tree::findTreeResource(BASE_TREE_RESOURCE_ID));
}

}  // namespace auto_apms_mission

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_mission::MissionOrchestrator)