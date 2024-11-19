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

#include "auto_apms_behavior_tree/build_handler.hpp"
#include "auto_apms_behavior_tree/exceptions.hpp"
#include "auto_apms_mission/mission_configuration.hpp"
#include "auto_apms_util/exceptions.hpp"

namespace auto_apms_mission
{

const std::string NOMINAL_MISSION_EXECUTOR_NODE_NAME = "nominal_mission";
const std::string SAFETY_MONITOR_EXECUTOR_NODE_NAME = "safety_monitor";
const std::string CONTINGENCY_HANDLER_EXECUTOR_NODE_NAME = "contingency_handler";
const std::string BUILTIN_NODES_START_EXECUTOR_NAME = "StartExecutor";
const std::string BUILTIN_NODES_HALT_EXECUTOR_NAME = "HaltExecutor";
const std::string BUILTIN_NODES_RESUME_EXECUTOR_NAME = "ResumeExecutor";

class MissionBuildHandler : public auto_apms_behavior_tree::TreeBuildHandler
{
public:
  MissionBuildHandler(rclcpp::Node::SharedPtr node_ptr);

private:
  bool setRequest(const std::string & request) override final;

  void handleBuild(TreeBuilder & builder, TreeBlackboard & bb) override final;

  TreeBuilder::PortValues createStartExecutorPorts(
    const std::string & tree_identity, const std::string & executor_name) const;

  const auto_apms_behavior_tree::core::NodeManifest builtin_nodes_manifest_;
  MissionConfiguration mission_config_;
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

MissionBuildHandler::MissionBuildHandler(rclcpp::Node::SharedPtr node_ptr)
: TreeBuildHandler(node_ptr),
  builtin_nodes_manifest_(
    auto_apms_behavior_tree::core::NodeManifest::fromResourceIdentity("auto_apms_behavior_tree::builtin_nodes"))
{
}

bool MissionBuildHandler::setRequest(const std::string & request)
{
  try {
    mission_config_ = MissionConfiguration::fromResourceIdentity(request);
  } catch (const auto_apms_util::exceptions::ResourceIdentityFormatError & e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
    return false;
  } catch (const auto_apms_util::exceptions::ResourceError & e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
    return false;
  }
  return true;
}

void MissionBuildHandler::handleBuild(TreeBuilder & builder, TreeBlackboard & /*bb*/)
{
  auto insert_sub_trees = [this, &builder](
                            TreeBuilder::ElementPtr parent, const std::vector<std::string> & tree_identities) {
    for (const std::string & tree_identity : tree_identities) {
      auto_apms_behavior_tree::core::TreeResource tree_resource(tree_identity);
      const std::string root_tree = tree_resource.getRootTreeName(TreeBuilder::ROOT_TREE_ATTRIBUTE_NAME);
      if (root_tree.empty()) {
        throw auto_apms_behavior_tree::exceptions::TreeBuildError(
          "Cannot add subtree for tree resource '" + tree_identity +
          "', because no root tree was specified in the XML file.");
      }
      builder.mergeTreesFromResource(tree_resource);
      builder.insertSubTree(parent, root_tree);
    }
  };

  // Bringup sequence
  if (!mission_config_.bringup.empty()) {
    TreeBuilder::ElementPtr bringup_tree = builder.findTree("BringUp");
    bringup_tree->DeleteChildren();
    insert_sub_trees(bringup_tree, mission_config_.bringup);
  }

  // Mission sequence
  if (!mission_config_.mission.empty()) {
    TreeBuilder::ElementPtr mission_sequence = builder.findTree("RunMission")->FirstChildElement("SequenceWithMemory");
    mission_sequence->DeleteChildren();
    for (const std::string & tree_identity : mission_config_.mission) {
      builder.addNodePortValues(
        builder.insertNode(
          mission_sequence, BUILTIN_NODES_START_EXECUTOR_NAME,
          builtin_nodes_manifest_[BUILTIN_NODES_START_EXECUTOR_NAME]),
        createStartExecutorPorts(tree_identity, NOMINAL_MISSION_EXECUTOR_NODE_NAME), true);
    }
  }

  // Contingency handling
  if (!mission_config_.contingency.empty()) {
    TreeBuilder::ElementPtr contingency_sequence =
      builder.findFirstNode(builder.findTree("MonitorContingencies"), "SequenceWithMemory");
    contingency_sequence->DeleteChildren();
    for (const auto & [handle_identity, monitors] : mission_config_.contingency) {
      TreeBuilder::ElementPtr fallback = builder.insertNode(contingency_sequence, "Fallback");

      // Detect contingencies
      TreeBuilder::ElementPtr detect_sequence =
        builder.insertNode(builder.insertNode(fallback, "Inverter"), "Sequence");
      detect_sequence->SetAttribute(TreeBuilder::NODE_INSTANCE_NAME_ATTRIBUTE_NAME, "DetectContingency");
      for (const std::string & monitor_identity : monitors) {
        builder.addNodePortValues(
          builder.insertNode(
            detect_sequence, BUILTIN_NODES_START_EXECUTOR_NAME,
            builtin_nodes_manifest_[BUILTIN_NODES_START_EXECUTOR_NAME]),
          createStartExecutorPorts(monitor_identity, SAFETY_MONITOR_EXECUTOR_NODE_NAME), true);
      }

      // Handle contingencies
      TreeBuilder::ElementPtr handle_sequence = builder.insertNode(fallback, "Sequence");
      handle_sequence->SetAttribute(TreeBuilder::NODE_INSTANCE_NAME_ATTRIBUTE_NAME, "HandleContingency");
      builder.addNodePortValues(
        builder.insertNode(
          handle_sequence, BUILTIN_NODES_HALT_EXECUTOR_NAME, builtin_nodes_manifest_[BUILTIN_NODES_HALT_EXECUTOR_NAME]),
        {{"action_name",
          NOMINAL_MISSION_EXECUTOR_NODE_NAME + _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_COMMAND_ACTION_NAME_SUFFIX}},
        true);
      builder.addNodePortValues(
        builder.insertNode(
          handle_sequence, BUILTIN_NODES_START_EXECUTOR_NAME,
          builtin_nodes_manifest_[BUILTIN_NODES_START_EXECUTOR_NAME]),
        createStartExecutorPorts(handle_identity, CONTINGENCY_HANDLER_EXECUTOR_NODE_NAME), true);
      builder.addNodePortValues(
        builder.insertNode(
          handle_sequence, BUILTIN_NODES_RESUME_EXECUTOR_NAME,
          builtin_nodes_manifest_[BUILTIN_NODES_RESUME_EXECUTOR_NAME]),
        {{"action_name",
          NOMINAL_MISSION_EXECUTOR_NODE_NAME + _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_COMMAND_ACTION_NAME_SUFFIX}},
        true);
    }
  }

  // Shutdown sequence
  if (!mission_config_.shutdown.empty()) {
    TreeBuilder::ElementPtr shutdown_tree = builder.findTree("ShutDown");
    shutdown_tree->DeleteChildren();
    insert_sub_trees(shutdown_tree, mission_config_.shutdown);
  }
}

MissionBuildHandler::TreeBuilder::PortValues MissionBuildHandler::createStartExecutorPorts(
  const std::string & tree_identity, const std::string & executor_name) const
{
  TreeBuilder::PortValues start_executor_ports;
  start_executor_ports["build_request"] = tree_identity;
  start_executor_ports["build_handler"] = "auto_apms_behavior_tree::TreeResourceBuildHandler";
  start_executor_ports["attach"] = "true";
  start_executor_ports["clear_blackboard"] = "false";
  start_executor_ports["action_name"] = executor_name + _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_START_ACTION_NAME_SUFFIX;
  return start_executor_ports;
}

}  // namespace auto_apms_mission

AUTO_APMS_BEHAVIOR_TREE_REGISTER_BUILD_HANDLER(auto_apms_mission::MissionBuildHandler)