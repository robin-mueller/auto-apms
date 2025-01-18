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
#include "auto_apms_mission/mission_configuration.hpp"

/**
 * @defgroup auto_apms_mission Mission Design
 * @brief Useful concepts that allow the user to quickly and easily configure custom missions including fallback
 * mechanisms if something goes wrong.
 */

/**
 * @ingroup auto_apms_mission
 * @brief Mission design utilities incorporating behavior trees to model the complexity of arbitrary operations.
 */
namespace auto_apms_mission
{

namespace model = auto_apms_behavior_tree::model;

/**
 * @ingroup auto_apms_mission
 * @brief Base class for behavior tree build handlers that are used to configure missions including fallback mechanisms.
 */
class MissionBuildHandlerBase : public auto_apms_behavior_tree::TreeBuildHandler
{
public:
  static const std::string ORCHESTRATOR_EXECUTOR_NAME;
  static const std::string MISSION_EXECUTOR_NAME;
  static const std::string EVENT_MONITOR_EXECUTOR_NAME;
  static const std::string EVENT_HANDLER_EXECUTOR_NAME;

  MissionBuildHandlerBase(rclcpp::Node::SharedPtr ros_node_ptr, NodeLoader::SharedPtr tree_node_loader_ptr);

private:
  bool setBuildRequest(
    const std::string & build_request, const NodeManifest & node_manifest,
    const std::string & root_tree_name) override final;

  TreeDocument::TreeElement buildTree(TreeDocument & doc, TreeBlackboard & bb) override final;

protected:
  /* Virtual methods */

  virtual void buildBringUp(TreeDocument::TreeElement & sub_tree, const std::vector<TreeResource::Identity> & trees);

  virtual void buildMission(
    TreeDocument::TreeElement & sub_tree, const std::vector<TreeResource::Identity> & trees) = 0;

  virtual void buildEventMonitor(
    TreeDocument::TreeElement & sub_tree,
    const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & contingencies,
    const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & emergencies);

  virtual void buildContingencyHandling(
    TreeDocument::TreeElement & sub_tree,
    const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & contingencies);

  virtual void buildEmergencyHandling(
    TreeDocument::TreeElement & sub_tree,
    const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & emergencies);

  virtual void buildShutDown(TreeDocument::TreeElement & sub_tree, const std::vector<TreeResource::Identity> & trees);

  virtual void configureOrchestratorRootBlackboard(TreeBlackboard & bb);

private:
  MissionConfiguration mission_config_;
};

}  // namespace auto_apms_mission