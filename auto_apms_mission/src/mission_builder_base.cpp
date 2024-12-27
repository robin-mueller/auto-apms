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

  TreeDocument::TreeElement is_contingency_tree = builder.getTree("IsContingency").removeChildren();
  TreeDocument::TreeElement is_emergency_tree = builder.getTree("IsEmergency").removeChildren();

  if (mission_config_.contingency.empty()) {
    is_contingency_tree.insertNode<model::AlwaysFailure>();
  } else {
    model::Fallback fallback = is_contingency_tree.insertNode<model::Fallback>();
    for (const auto & [event_id, _] : mission_config_.contingency) {
      fallback.insertNode<model::ScriptCondition>().set_code("event_id == '" + event_id.str() + "'");
    }
  }

  if (mission_config_.emergency.empty()) {
    is_emergency_tree.insertNode<model::AlwaysFailure>();
  } else {
    model::Fallback fallback = is_emergency_tree.insertNode<model::Fallback>();
    for (const auto & [event_id, _] : mission_config_.emergency) {
      fallback.insertNode<model::ScriptCondition>().set_code("event_id == '" + event_id.str() + "'");
    }
  }

  // Bring up
  if (!mission_config_.bringup.empty()) {
    RCLCPP_DEBUG(logger_, "Creating bringup subtree.");
    TreeDocument::TreeElement bringup_tree = builder.getTree("BringUp");
    buildBringUp(bringup_tree, mission_config_.bringup);
    if (const BT::Result res = bringup_tree.verify(); !res) {
      throw auto_apms_behavior_tree::exceptions::TreeBuildHandlerError("Bringup tree is not valid: " + res.error());
    }
  }

  // Mission
  RCLCPP_DEBUG(logger_, "Creating mission subtree.");
  TreeDocument::TreeElement mission_tree = builder.getTree("RunMission");
  buildMission(mission_tree, mission_config_.mission);
  if (const BT::Result res = mission_tree.verify(); !res) {
    throw auto_apms_behavior_tree::exceptions::TreeBuildHandlerError("Mission tree is not valid: " + res.error());
  }

  // Event monitor
  if (!mission_config_.contingency.empty() || !mission_config_.emergency.empty()) {
    RCLCPP_DEBUG(logger_, "Creating event monitor subtree.");
    TreeDocument::TreeElement event_monitor_tree = builder.getTree("MonitorEvents");
    buildEventMonitor(event_monitor_tree, mission_config_.contingency, mission_config_.emergency);
    if (const BT::Result res = event_monitor_tree.verify(); !res) {
      throw auto_apms_behavior_tree::exceptions::TreeBuildHandlerError(
        "Event monitor tree is not valid: " + res.error());
    }
  }

  // Contingency Handling
  if (!mission_config_.contingency.empty()) {
    RCLCPP_DEBUG(logger_, "Creating contingency handler subtree.");
    TreeDocument::TreeElement contingency_tree = builder.getTree("HandleContingency");
    buildContingencyHandling(contingency_tree, mission_config_.contingency);
    if (const BT::Result res = contingency_tree.verify(); !res) {
      throw auto_apms_behavior_tree::exceptions::TreeBuildHandlerError(
        "Contingency handling tree is not valid: " + res.error());
    }
  }

  // Emergency Handling
  if (!mission_config_.emergency.empty()) {
    RCLCPP_DEBUG(logger_, "Creating emergency handler subtree.");
    TreeDocument::TreeElement emergency_tree = builder.getTree("HandleEmergency");
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
    buildShutDown(shutdown_tree, mission_config_.shutdown);
    if (const BT::Result res = shutdown_tree.verify(); !res) {
      throw auto_apms_behavior_tree::exceptions::TreeBuildHandlerError("Shutdown tree is not valid: " + res.error());
    }
  }

  // Write tree for debugging purposes
  // builder.writeToFile("/home/robin/Desktop/px4-ros2-env/src/dep/auto-apms/test.xml");

  return root_tree;
}

void MissionBuildHandlerBase::buildBringUp(
  TreeDocument::TreeElement & sub_tree, const std::vector<TreeResource::Identity> & trees)
{
  sub_tree.removeChildren();
  for (const TreeResource::Identity & r : trees) {
    sub_tree.insertTreeFromResource(r);
  }
}

void MissionBuildHandlerBase::buildEventMonitor(
  TreeDocument::TreeElement & sub_tree,
  const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & contingencies,
  const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & emergencies)
{
  std::vector<TreeResource::Identity> event_ids;

  // Emergencies have higher priority than contingencies (they are inserted to the vector first)
  for (const auto & [event_id, _] : emergencies) {
    event_ids.push_back(event_id);
  }
  for (const auto & [event_id, _] : contingencies) {
    if (!auto_apms_util::contains(event_ids, event_id)) event_ids.push_back(event_id);
  }

  model::Fallback fallback = sub_tree.removeChildren().insertNode<model::Fallback>();
  for (const TreeResource::Identity & r : event_ids) {
    fallback.insertTreeFromResource(r).setConditionalScript(BT::PostCond::ON_SUCCESS, "event_id := '" + r.str() + "'");
  }
}

void MissionBuildHandlerBase::buildContingencyHandling(
  TreeDocument::TreeElement & sub_tree,
  const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & contingencies)
{
  model::ReactiveFallback fallback = sub_tree.removeChildren().insertNode<model::ReactiveFallback>();

  // At the highest priority (first child) we acknowledge when the event is reset (e.g. we abort the current action and
  // resume)
  fallback.insertNode<model::AlwaysFailure>()
    .setName("ResetEventHandler")
    .setConditionalScript(BT::PreCond::SUCCESS_IF, "event_id == ''");

  for (const auto & [event_id, handler_id] : contingencies) {
    model::AsyncSequence seq =
      fallback.insertNode<model::AsyncSequence>().setName("EventHandler (" + handler_id.str() + ")");
    seq.insertNode<model::ScriptCondition>().set_code("event_id == '" + event_id.str() + "'");
    seq.insertTreeFromResource(handler_id);
  }
}

void MissionBuildHandlerBase::buildEmergencyHandling(
  TreeDocument::TreeElement & sub_tree,
  const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & emergencies)
{
  model::ReactiveFallback fallback = sub_tree.removeChildren().insertNode<model::ReactiveFallback>();
  for (const auto & [event_id, handler_id] : emergencies) {
    model::AsyncSequence seq =
      fallback.insertNode<model::AsyncSequence>().setName("EventHandler (" + handler_id.str() + ")");
    seq.insertNode<model::ScriptCondition>().set_code("event_id == '" + event_id.str() + "'");
    seq.insertTreeFromResource(handler_id);
  }
}

void MissionBuildHandlerBase::buildShutDown(
  TreeDocument::TreeElement & sub_tree, const std::vector<TreeResource::Identity> & trees)
{
  sub_tree.removeChildren();
  for (const TreeResource::Identity & r : trees) {
    sub_tree.insertTreeFromResource(r);
  }
}

void MissionBuildHandlerBase::configureOrchestratorRootBlackboard(TreeBlackboard & /*bb*/) {}

}  // namespace auto_apms_mission