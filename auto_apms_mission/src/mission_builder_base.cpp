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

const std::string MissionBuilderBase::ORCHESTRATOR_EXECUTOR_NAME = _AUTO_APMS_MISSION__ORCHESTRATOR_EXECUTOR_NAME;
const std::string MissionBuilderBase::MISSION_EXECUTOR_NAME = _AUTO_APMS_MISSION__MISSION_EXECUTOR_NAME;
const std::string MissionBuilderBase::EVENT_MONITOR_EXECUTOR_NAME = _AUTO_APMS_MISSION__EVENT_MONITOR_EXECUTOR_NAME;
const std::string MissionBuilderBase::EVENT_HANDLER_EXECUTOR_NAME = _AUTO_APMS_MISSION__EVENT_HANDLER_EXECUTOR_NAME;

MissionBuilderBase::MissionBuilderBase(rclcpp::Node::SharedPtr node_ptr)
: TreeBuildHandler(node_ptr),
  builtin_nodes_(
    auto_apms_behavior_tree::core::NodeManifest::fromResourceIdentity("auto_apms_behavior_tree::builtin_nodes"))
{
}

MissionBuilderBase::NodeElement MissionBuilderBase::insertStartTreeExecutorNode(
  NodeElement & parent, const StartTreeExecutorRequest & request, const std::string & executor_ros_node)
{
  NodeElement ele = parent.loadAndInsertNode(
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_START_EXECUTOR,
    builtin_nodes_[_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_START_EXECUTOR]);

  NodeElement::PortValues ports;
  ports["build_request"] = request.build_request;
  ports["build_handler"] = request.build_handler;
  ports["root_tree"] = request.root_tree;
  ports["attach"] = request.attach ? "true" : "false";
  ports["node_overrides"] = request.node_overrides;
  ports["clear_blackboard"] = request.clear_blackboard ? "true" : "false";
  ports["action_name"] = executor_ros_node + _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_START_ACTION_NAME_SUFFIX;
  return ele.setPorts(ports, true);
}

bool MissionBuilderBase::setBuildRequest(const std::string & build_request, const std::string & root_tree_name)
{
  try {
    mission_config_ = MissionConfiguration::fromResourceIdentity(build_request);
  } catch (const auto_apms_util::exceptions::ResourceIdentityFormatError & e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
    return false;
  } catch (const auto_apms_util::exceptions::ResourceError & e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
    return false;
  }
  if (!root_tree_name.empty()) {
    RCLCPP_WARN(
      logger_, "Argument root_tree_name is not empty. MissionBuilderBase doesn't support custom root tree names.");
  }
  return true;
}

MissionBuilderBase::TreeElement MissionBuilderBase::buildTree(TreeBuilder & builder, TreeBlackboard & bb)
{
  // Load orchestrator tree
  TreeElement root_tree =
    builder.newTreeFromResource("auto_apms_mission::orchestrator_base::MissionOrchestrator").makeRoot();

  std::vector<TreeResource> bringup_trees;
  std::vector<TreeResource> mission_trees;
  std::vector<TreeResource> shutdown_trees;
  for (const std::string & str : mission_config_.bringup) bringup_trees.push_back({str});
  for (const std::string & str : mission_config_.mission) mission_trees.push_back({str});
  for (const std::string & str : mission_config_.shutdown) shutdown_trees.push_back({str});

  // Bring up
  NodeElement bringup_sequence = builder.getTree("BringUp").getFirstNode("SequenceWithMemory").removeChildren();
  buildBringUp(bringup_sequence, bringup_trees);
  if (!bringup_sequence.hasChildren()) bringup_sequence.insertNode("AlwaysSuccess");

  // Run mission
  NodeElement mission_sequence = builder.getTree("RunMission").getFirstNode("SequenceWithMemory").removeChildren();
  buildMission(mission_sequence, mission_trees);
  if (!mission_sequence.hasChildren()) mission_sequence.insertNode("AlwaysSuccess");

  // buildEventMonitor(builder);

  // buildEventHandler(builder);

  // Shut down
  NodeElement shutdown_sequence = builder.getTree("ShutDown").getFirstNode("SequenceWithMemory").removeChildren();
  buildShutDown(shutdown_sequence, shutdown_trees);
  if (!shutdown_sequence.hasChildren()) shutdown_sequence.insertNode("AlwaysSuccess");

  return root_tree;
}

void MissionBuilderBase::buildBringUp(NodeElement & /*sequence*/, TreeResourceVector /*trees*/) {}

void MissionBuilderBase::buildEventMonitor(NodeElement & /*sequence*/, TreeResourceVector /*trees*/) {}

void MissionBuilderBase::buildEventHandler(NodeElement & /*sequence*/, TreeResourceVector /*trees*/) {}

void MissionBuilderBase::buildShutDown(NodeElement & /*sequence*/, TreeResourceVector /*trees*/) {}

void MissionBuilderBase::configureOrchestratorBlackboard(TreeBlackboard & /*bb*/) {}

}  // namespace auto_apms_mission