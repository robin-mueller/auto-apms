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
  TreeBuilder & builder, TreeBlackboard & /*bb*/)
{
  using namespace auto_apms_behavior_tree::model;

  // Load orchestrator tree
  TreeDocument::TreeElement root_tree =
    builder.newTreeFromResource("auto_apms_mission::orchestrator_base::MissionOrchestrator").makeRoot();

  std::vector<TreeResource> bringup_trees_vec;
  std::vector<TreeResource> mission_trees_vec;
  std::vector<TreeResource> shutdown_trees_vec;
  for (const std::string & str : mission_config_.bringup) bringup_trees_vec.push_back({str});
  for (const std::string & str : mission_config_.mission) mission_trees_vec.push_back({str});
  for (const std::string & str : mission_config_.shutdown) shutdown_trees_vec.push_back({str});

  // Bring up
  TreeDocument::TreeElement bringup_tree = builder.getTree("BringUp");
  SequenceWithMemory bringup_sequence = bringup_tree.getFirstNode<SequenceWithMemory>().removeChildren();
  buildBringUp(bringup_sequence, bringup_trees_vec);
  if (bringup_sequence.hasChildren()) {
    if (const BT::Result res = bringup_tree.verify(); !res) {
      throw auto_apms_behavior_tree::exceptions::TreeBuildHandlerError("Bringup tree is not valid: " + res.error());
    }
  } else {
    bringup_sequence.insertNode<AlwaysSuccess>();
  }

  // Mission
  TreeDocument::TreeElement mission_tree = builder.getTree("RunMission");
  SequenceWithMemory mission_sequence = mission_tree.getFirstNode<SequenceWithMemory>().removeChildren();
  buildMission(mission_sequence, mission_trees_vec);
  if (mission_sequence.hasChildren()) {
    if (const BT::Result res = mission_tree.verify(); !res) {
      throw auto_apms_behavior_tree::exceptions::TreeBuildHandlerError("Mission tree is not valid: " + res.error());
    }
  } else {
    mission_sequence.insertNode<AlwaysSuccess>();
  }

  // buildEventMonitor(builder);

  // buildEventHandler(builder);

  // Shut down
  TreeDocument::TreeElement shutdown_tree = builder.getTree("ShutDown");
  SequenceWithMemory shutdown_sequence = shutdown_tree.getFirstNode<SequenceWithMemory>().removeChildren();
  buildShutDown(shutdown_sequence, shutdown_trees_vec);
  if (shutdown_sequence.hasChildren()) {
    if (const BT::Result res = shutdown_tree.verify(); !res) {
      throw auto_apms_behavior_tree::exceptions::TreeBuildHandlerError("Shutdown tree is not valid: " + res.error());
    }
  } else {
    shutdown_sequence.insertNode<AlwaysSuccess>();
  }

  return root_tree;
}

void MissionBuildHandlerBase::buildBringUp(
  model::SequenceWithMemory & /*sequence*/, const std::vector<TreeResource> & /*trees*/)
{
}

void MissionBuildHandlerBase::buildEventMonitor(
  model::SequenceWithMemory & /*sequence*/, const std::vector<TreeResource> & /*trees*/)
{
}

void MissionBuildHandlerBase::buildEventHandler(
  model::SequenceWithMemory & /*sequence*/, const std::vector<TreeResource> & /*trees*/)
{
}

void MissionBuildHandlerBase::buildShutDown(
  model::SequenceWithMemory & /*sequence*/, const std::vector<TreeResource> & /*trees*/)
{
}

void MissionBuildHandlerBase::configureOrchestratorBlackboard(TreeBlackboard & /*bb*/) {}

}  // namespace auto_apms_mission