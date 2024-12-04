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

#pragma once

#include "auto_apms_behavior_tree/build_handler.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_resource.hpp"
#include "auto_apms_mission/mission_configuration.hpp"

namespace auto_apms_mission
{

namespace model = auto_apms_behavior_tree::model;

class MissionBuilderBase : public auto_apms_behavior_tree::TreeBuildHandler
{
public:
  using TreeResourceVector = const std::vector<TreeResource> &;

  static const std::string ORCHESTRATOR_EXECUTOR_NAME;
  static const std::string MISSION_EXECUTOR_NAME;
  static const std::string EVENT_MONITOR_EXECUTOR_NAME;
  static const std::string EVENT_HANDLER_EXECUTOR_NAME;

  MissionBuilderBase(rclcpp::Node::SharedPtr node_ptr);

private:
  bool setBuildRequest(const std::string & build_request, const std::string & root_tree_name) override final;

  TreeElement buildTree(TreeBuilder & builder, TreeBlackboard & bb) override final;

private:
  /* Virtual methods */

  virtual void buildBringUp(model::SequenceWithMemory & sequence, TreeResourceVector trees);

  virtual void buildMission(model::SequenceWithMemory & sequence, TreeResourceVector trees) = 0;

  virtual void buildEventMonitor(model::SequenceWithMemory & sequence, TreeResourceVector trees);

  virtual void buildEventHandler(model::SequenceWithMemory & sequence, TreeResourceVector trees);

  virtual void buildShutDown(model::SequenceWithMemory & sequence, TreeResourceVector trees);

  virtual void configureOrchestratorBlackboard(TreeBlackboard & bb);

private:
  MissionConfiguration mission_config_;
};

}  // namespace auto_apms_mission