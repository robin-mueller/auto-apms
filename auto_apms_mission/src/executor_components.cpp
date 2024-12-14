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

#include "auto_apms_behavior_tree/executor/executor_node.hpp"

namespace auto_apms_mission
{

class OrchestratorExecutor : public auto_apms_behavior_tree::TreeExecutorNode
{
public:
  explicit OrchestratorExecutor(rclcpp::NodeOptions options)
  : TreeExecutorNode(
      _AUTO_APMS_MISSION__ORCHESTRATOR_EXECUTOR_NAME,
      Options(options).setDefaultBuildHandler("auto_apms_mission::SingleNodeMissionBuildHandler"))
  {
  }
};

class MissionExecutor : public auto_apms_behavior_tree::TreeExecutorNode
{
public:
  explicit MissionExecutor(rclcpp::NodeOptions options)
  : TreeExecutorNode(_AUTO_APMS_MISSION__MISSION_EXECUTOR_NAME, options)
  {
  }
};

class EventMonitorExecutor : public auto_apms_behavior_tree::TreeExecutorNode
{
public:
  explicit EventMonitorExecutor(rclcpp::NodeOptions options)
  : TreeExecutorNode(_AUTO_APMS_MISSION__EVENT_MONITOR_EXECUTOR_NAME, options)
  {
  }
};

class EventHandlerExecutor : public auto_apms_behavior_tree::TreeExecutorNode
{
public:
  explicit EventHandlerExecutor(rclcpp::NodeOptions options)
  : TreeExecutorNode(_AUTO_APMS_MISSION__EVENT_HANDLER_EXECUTOR_NAME, options)
  {
  }
};

}  // namespace auto_apms_mission

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_mission::OrchestratorExecutor)
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_mission::MissionExecutor)
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_mission::EventMonitorExecutor)
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_mission::EventHandlerExecutor)