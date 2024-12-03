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

#include "auto_apms_behavior_tree/exceptions.hpp"
#include "auto_apms_mission/mission_builder_base.hpp"
#include "auto_apms_util/exceptions.hpp"

namespace auto_apms_mission
{

class MissionBuilder : public MissionBuilderBase
{
public:
  using MissionBuilderBase::MissionBuilderBase;

private:
  void buildBringUp(model::SequenceWithMemory & sequence, TreeResourceVector trees) override final;

  void buildMission(model::SequenceWithMemory & sequence, TreeResourceVector trees) override final;

  void buildShutDown(model::SequenceWithMemory & sequence, TreeResourceVector trees) override final;
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

// void MissionBuilder::buildTree(TreeBuilder & builder, TreeBlackboard & /*bb*/)
// {

//   // Mission sequence
//   if (!mission_config_.mission.empty()) {
//     TreeBuilder::ElementPtr mission_sequence =
//     builder.findTree("RunMission")->FirstChildElement("SequenceWithMemory"); mission_sequence->DeleteChildren(); for
//     (const std::string & tree_identity : mission_config_.mission) {
//       builder.addNodePortValues(
//         builder.insertNode(
//           mission_sequence, BUILTIN_NODES_START_EXECUTOR_NAME,
//           builtin_nodes_manifest_[BUILTIN_NODES_START_EXECUTOR_NAME]),
//         createStartExecutorPorts(tree_identity, NOMINAL_MISSION_EXECUTOR_NODE_NAME), true);
//     }
//   }

//   // Contingency handling
//   if (!mission_config_.contingency.empty()) {
//     TreeBuilder::ElementPtr contingency_sequence =
//       builder.findFirstNode(builder.findTree("MonitorContingencies"), "SequenceWithMemory");
//     contingency_sequence->DeleteChildren();
//     for (const auto & [handle_identity, monitors] : mission_config_.contingency) {
//       TreeBuilder::ElementPtr fallback = builder.insertNode(contingency_sequence, "Fallback");

//       // Detect contingencies
//       TreeBuilder::ElementPtr detect_sequence =
//         builder.insertNode(builder.insertNode(fallback, "Inverter"), "Sequence");
//       detect_sequence->SetAttribute(TreeBuilder::NODE_INSTANCE_NAME_ATTRIBUTE_NAME, "DetectContingency");
//       for (const std::string & monitor_identity : monitors) {
//         builder.addNodePortValues(
//           builder.insertNode(
//             detect_sequence, BUILTIN_NODES_START_EXECUTOR_NAME,
//             builtin_nodes_manifest_[BUILTIN_NODES_START_EXECUTOR_NAME]),
//           createStartExecutorPorts(monitor_identity, SAFETY_MONITOR_EXECUTOR_NODE_NAME), true);
//       }

//       // Handle contingencies
//       TreeBuilder::ElementPtr handle_sequence = builder.insertNode(fallback, "Sequence");
//       handle_sequence->SetAttribute(TreeBuilder::NODE_INSTANCE_NAME_ATTRIBUTE_NAME, "HandleContingency");
//       builder.addNodePortValues(
//         builder.insertNode(
//           handle_sequence, BUILTIN_NODES_HALT_EXECUTOR_NAME,
//           builtin_nodes_manifest_[BUILTIN_NODES_HALT_EXECUTOR_NAME]),
//         {{"action_name",
//           NOMINAL_MISSION_EXECUTOR_NODE_NAME + _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_COMMAND_ACTION_NAME_SUFFIX}},
//         true);
//       builder.addNodePortValues(
//         builder.insertNode(
//           handle_sequence, BUILTIN_NODES_START_EXECUTOR_NAME,
//           builtin_nodes_manifest_[BUILTIN_NODES_START_EXECUTOR_NAME]),
//         createStartExecutorPorts(handle_identity, CONTINGENCY_HANDLER_EXECUTOR_NODE_NAME), true);
//       builder.addNodePortValues(
//         builder.insertNode(
//           handle_sequence, BUILTIN_NODES_RESUME_EXECUTOR_NAME,
//           builtin_nodes_manifest_[BUILTIN_NODES_RESUME_EXECUTOR_NAME]),
//         {{"action_name",
//           NOMINAL_MISSION_EXECUTOR_NODE_NAME + _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_COMMAND_ACTION_NAME_SUFFIX}},
//         true);
//     }
//   }
// }

void MissionBuilder::buildBringUp(model::SequenceWithMemory & sequence, TreeResourceVector trees)
{
  for (const TreeResource & r : trees) {
    sequence.insertTreeFromResource(r);
  }
}

void MissionBuilder::buildMission(model::SequenceWithMemory & sequence, TreeResourceVector trees)
{
  for (const TreeResource & r : trees) {
    sequence.insertNode<model::StartExecutor>();
  }
}

void MissionBuilder::buildShutDown(model::SequenceWithMemory & sequence, TreeResourceVector trees)
{
  for (const TreeResource & r : trees) {
    sequence.insertTreeFromResource(r);
  }
}

}  // namespace auto_apms_mission

AUTO_APMS_BEHAVIOR_TREE_REGISTER_BUILD_HANDLER(auto_apms_mission::MissionBuilder)