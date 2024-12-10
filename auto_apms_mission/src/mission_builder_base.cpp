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

#include "auto_apms_mission/mission_builder_base.hpp"

#include "auto_apms_util/exceptions.hpp"

namespace auto_apms_mission
{

const std::string MissionBuildHandlerBase::ORCHESTRATOR_EXECUTOR_NAME = _AUTO_APMS_MISSION__ORCHESTRATOR_EXECUTOR_NAME;
const std::string MissionBuildHandlerBase::MISSION_EXECUTOR_NAME = _AUTO_APMS_MISSION__MISSION_EXECUTOR_NAME;
const std::string MissionBuildHandlerBase::EVENT_MONITOR_EXECUTOR_NAME =
  _AUTO_APMS_MISSION__EVENT_MONITOR_EXECUTOR_NAME;
const std::string MissionBuildHandlerBase::EVENT_HANDLER_EXECUTOR_NAME =
  _AUTO_APMS_MISSION__EVENT_HANDLER_EXECUTOR_NAME;

MissionBuildHandlerBase::MissionBuildHandlerBase(
  rclcpp::Node::SharedPtr ros_node_ptr, NodeLoader::SharedPtr tree_node_loader_ptr)
: TreeBuildHandler("mission_builder", ros_node_ptr, tree_node_loader_ptr)
{
}

bool MissionBuildHandlerBase::setBuildRequest(
  const std::string & build_request, const NodeManifest & /*node_manifest*/, const std::string & root_tree_name)
{
  try {
    mission_config_ = MissionConfiguration::fromResourceIdentity(build_request);
  } catch (const auto_apms_util::exceptions::ResourceIdentityFormatError & e) {
    RCLCPP_WARN(logger_, "%s", e.what());
    return false;
  } catch (const auto_apms_util::exceptions::ResourceError & e) {
    RCLCPP_WARN(logger_, "%s", e.what());
    return false;
  }
  if (!root_tree_name.empty()) {
    RCLCPP_WARN(logger_, "Argument root_tree_name is not empty. Custom root tree names are not allowed.");
    return false;
  }
  return true;
}

MissionBuildHandlerBase::TreeDocument::TreeElement MissionBuildHandlerBase::buildTree(
  TreeBuilder & builder, TreeBlackboard & bb)
{
  using namespace auto_apms_behavior_tree::model;

  // Load orchestrator tree
  RCLCPP_DEBUG(logger_, "Loading orchestrator tree.");
  TreeDocument::TreeElement root_tree =
    builder.newTreeFromResource("auto_apms_mission::orchestrator_base::MissionOrchestrator").makeRoot();

  RCLCPP_DEBUG(logger_, "Configuring orchestrator root blackboard.");
  configureOrchestratorRootBlackboard(bb);

  model::Fallback is_contingency_fallback =
    builder.getTree("IsContingency").removeChildren().insertNode<model::Fallback>();
  for (const auto & [event_id, _] : mission_config_.contingency) {
    is_contingency_fallback.insertNode<model::ScriptCondition>().set_code("event_id == '" + event_id.str() + "'");
  }
  model::Fallback is_emergency_fallback = builder.getTree("IsEmergency").removeChildren().insertNode<model::Fallback>();
  for (const auto & [event_id, _] : mission_config_.emergency) {
    is_emergency_fallback.insertNode<model::ScriptCondition>().set_code("event_id == '" + event_id.str() + "'");
  }

  // Bring up
  if (!mission_config_.bringup.empty()) {
    RCLCPP_DEBUG(logger_, "Creating bringup subtree.");
    TreeDocument::TreeElement bringup_tree = builder.getTree("BringUp");
    SequenceWithMemory bringup_sequence = bringup_tree.getFirstNode<SequenceWithMemory>().removeChildren();
    buildBringUp(bringup_sequence, mission_config_.bringup);
    if (const BT::Result res = bringup_tree.verify(); !res) {
      throw auto_apms_behavior_tree::exceptions::TreeBuildHandlerError("Bringup tree is not valid: " + res.error());
    }
  }

  // Mission
  RCLCPP_DEBUG(logger_, "Creating mission subtree.");
  TreeDocument::TreeElement mission_tree = builder.getTree("RunMission");
  SequenceWithMemory mission_sequence = mission_tree.getFirstNode<SequenceWithMemory>().removeChildren();
  buildMission(mission_sequence, mission_config_.mission);
  if (const BT::Result res = mission_tree.verify(); !res) {
    throw auto_apms_behavior_tree::exceptions::TreeBuildHandlerError("Mission tree is not valid: " + res.error());
  }

  // Event monitor
  if (!mission_config_.contingency.empty() || !mission_config_.emergency.empty()) {
    RCLCPP_DEBUG(logger_, "Creating event monitor subtree.");
    TreeDocument::TreeElement event_monitor_tree = builder.getTree("MonitorEvents").removeChildren();
    buildEventMonitor(event_monitor_tree, mission_config_.contingency, mission_config_.emergency);
    if (const BT::Result res = event_monitor_tree.verify(); !res) {
      throw auto_apms_behavior_tree::exceptions::TreeBuildHandlerError(
        "Event monitor tree is not valid: " + res.error());
    }
  }

  // Contingency Handling
  if (!mission_config_.contingency.empty()) {
    RCLCPP_DEBUG(logger_, "Creating contingency handler subtree.");
    TreeDocument::TreeElement contingency_tree = builder.getTree("HandleContingency").removeChildren();
    buildContingencyHandling(contingency_tree, mission_config_.contingency);
    if (const BT::Result res = contingency_tree.verify(); !res) {
      throw auto_apms_behavior_tree::exceptions::TreeBuildHandlerError(
        "Contingency handling tree is not valid: " + res.error());
    }
  }

  // Emergency Handling
  if (!mission_config_.emergency.empty()) {
    RCLCPP_DEBUG(logger_, "Creating emergency handler subtree.");
    TreeDocument::TreeElement emergency_tree = builder.getTree("HandleEmergency").removeChildren();
    buildEmergencyHandling(emergency_tree, mission_config_.emergency);
    if (const BT::Result res = emergency_tree.verify(); !res) {
      throw auto_apms_behavior_tree::exceptions::TreeBuildHandlerError(
        "Emergency handling tree is not valid: " + res.error());
    }
  }

  // Shut down
  if (!mission_config_.shutdown.empty()) {
    RCLCPP_DEBUG(logger_, "Creating shutdown subtree.");
    TreeDocument::TreeElement shutdown_tree = builder.getTree("ShutDown");
    SequenceWithMemory shutdown_sequence = shutdown_tree.getFirstNode<SequenceWithMemory>().removeChildren();
    buildShutDown(shutdown_sequence, mission_config_.shutdown);
    if (const BT::Result res = shutdown_tree.verify(); !res) {
      throw auto_apms_behavior_tree::exceptions::TreeBuildHandlerError("Shutdown tree is not valid: " + res.error());
    }
  }

  return root_tree;
}

void MissionBuildHandlerBase::buildBringUp(
  model::SequenceWithMemory & /*sequence*/, const std::vector<TreeResource::Identity> & /*trees*/)
{
}

void MissionBuildHandlerBase::buildEventMonitor(
  TreeDocument::TreeElement & /*sub_tree*/,
  const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & /*contingencies*/,
  const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & /*emergencies*/)
{
}

void MissionBuildHandlerBase::buildContingencyHandling(
  TreeDocument::TreeElement & /*sub_tree*/,
  const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & /*contingencies*/)
{
}

void MissionBuildHandlerBase::buildEmergencyHandling(
  TreeDocument::TreeElement & /*sub_tree*/,
  const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & /*emergencies*/)
{
}

void MissionBuildHandlerBase::buildShutDown(
  model::SequenceWithMemory & /*sequence*/, const std::vector<TreeResource::Identity> & /*trees*/)
{
}

void MissionBuildHandlerBase::configureOrchestratorRootBlackboard(TreeBlackboard & /*bb*/) {}

}  // namespace auto_apms_mission