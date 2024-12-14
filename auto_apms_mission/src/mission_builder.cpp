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
#include "auto_apms_util/container.hpp"

namespace auto_apms_mission
{

class SingleNodeMissionBuildHandler : public MissionBuildHandlerBase
{
public:
  using MissionBuildHandlerBase::MissionBuildHandlerBase;

private:
  void buildMission(
    TreeDocument::TreeElement & sub_tree, const std::vector<TreeResource::Identity> & trees) override final
  {
    sub_tree.removeChildren();
    for (const TreeResource::Identity & r : trees) {
      sub_tree.insertTreeFromResource(r);
    }
  }
};

class MultipleNodesMissionBuildHandler : public MissionBuildHandlerBase
{
public:
  using MissionBuildHandlerBase::MissionBuildHandlerBase;

private:
  void buildMission(
    TreeDocument::TreeElement & sub_tree, const std::vector<TreeResource::Identity> & trees) override final
  {
    TreeDocument doc;
    TreeDocument::TreeElement mission_tree = doc.newTree("Mission");
    TreeDocument::NodeElement mission_sequence = mission_tree.insertNode<model::SequenceWithMemory>();
    for (const TreeResource::Identity & r : trees) {
      mission_sequence.insertTreeFromResource(r);
    }

    sub_tree.removeChildren();
    auto_apms_behavior_tree::insertStartExecutorFromString(sub_tree, mission_tree)
      .setName("Mission")
      .set_attach(true)
      .set_clear_blackboard(true)
      .set_executor(MISSION_EXECUTOR_NAME);
  }

  void buildEventMonitor(
    TreeDocument::TreeElement & sub_tree,
    const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & contingencies,
    const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & emergencies) override final
  {
    TreeDocument doc;
    TreeDocument::TreeElement event_monitor_tree = doc.newTree(sub_tree);
    MissionBuildHandlerBase::buildEventMonitor(event_monitor_tree, contingencies, emergencies);
    event_monitor_tree.getFirstNode()
      .setName(event_monitor_tree.getName())
      .setConditionalScript(BT::PostCond::ALWAYS, "@event_id = event_id");
    TreeDocument::TreeElement main_tree = doc.newTree("MainTree");
    main_tree.insertNode<model::KeepRunningUntilFailure>().insertNode<model::ForceSuccess>().insertTree(
      event_monitor_tree);

    model::Fallback fallback = sub_tree.getFirstNode<model::Fallback>("DetectEvents").removeChildren();
    model::Sequence run_once_seq = fallback.insertNode<model::RunOnce>().insertNode<model::Sequence>();
    run_once_seq.insertNode<model::SetParameterString>()
      .set_node(EVENT_MONITOR_EXECUTOR_NAME)
      .set_parameter("bb.event_id")
      .set_value("");
    auto_apms_behavior_tree::insertStartExecutorFromString(run_once_seq, main_tree)
      .setName("EventMonitor")
      .set_attach(false)
      .set_clear_blackboard(false)
      .set_executor(EVENT_MONITOR_EXECUTOR_NAME);
    fallback.insertNode<model::GetParameterString>()
      .set_node(EVENT_MONITOR_EXECUTOR_NAME)
      .set_parameter("bb.event_id")
      .set_value("{event_id}");
  }

  void buildContingencyHandling(
    TreeDocument::TreeElement & sub_tree,
    const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & contingencies) override final
  {
    sub_tree.removeChildren();

    TreeDocument doc;
    TreeDocument::TreeElement new_tree = doc.newTree(sub_tree);
    model::Sequence seq = new_tree.insertNode<model::Sequence>();
    seq.insertNode<model::Script>().set_code("event_id := ''");  // Initial event_id value
    model::Parallel parallel = seq.insertNode<model::Parallel>().set_success_count(1);

    parallel.insertNode<model::KeepRunningUntilFailure>()
      .insertNode<model::GetParameterString>()
      .set_node(EVENT_MONITOR_EXECUTOR_NAME)
      .set_parameter("bb.event_id")
      .set_value("{event_id}");

    TreeDocument::TreeElement base_tree = doc.newTree("TempTree");
    MissionBuildHandlerBase::buildContingencyHandling(base_tree, contingencies);
    parallel.insertTree(base_tree).setConditionalScript(BT::PreCond::SKIP_IF, "event_id == ''");

    auto_apms_behavior_tree::insertStartExecutorFromString(sub_tree, new_tree)
      .setName("ContingencyHandler")
      .set_attach(true)
      .set_clear_blackboard(true)
      .set_executor(EVENT_HANDLER_EXECUTOR_NAME);
  }

  void buildEmergencyHandling(
    TreeDocument::TreeElement & sub_tree,
    const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & emergencies) override final
  {
    sub_tree.removeChildren();

    TreeDocument doc;
    TreeDocument::TreeElement new_tree = doc.newTree(sub_tree);
    model::Sequence seq = new_tree.insertNode<model::Sequence>();
    seq.insertNode<model::Script>().set_code("event_id := ''");  // Initial event_id value
    model::Parallel parallel = seq.insertNode<model::Parallel>().set_success_count(1);

    parallel.insertNode<model::KeepRunningUntilFailure>()
      .insertNode<model::GetParameterString>()
      .set_node(EVENT_MONITOR_EXECUTOR_NAME)
      .set_parameter("bb.event_id")
      .set_value("{event_id}");

    TreeDocument::TreeElement base_tree = doc.newTree("TempTree");
    MissionBuildHandlerBase::buildEmergencyHandling(base_tree, emergencies);
    parallel.insertTree(base_tree).setConditionalScript(BT::PreCond::SKIP_IF, "event_id == ''");

    auto_apms_behavior_tree::insertStartExecutorFromString(sub_tree, new_tree)
      .setName("EmergencyHandler")
      .set_attach(true)
      .set_clear_blackboard(true)
      .set_executor(EVENT_HANDLER_EXECUTOR_NAME);
  }

  void buildShutDown(TreeDocument::TreeElement & sub_tree, const std::vector<TreeResource::Identity> & trees)
  {
    TreeDocument doc;
    TreeDocument::TreeElement temp = doc.newTree("TempTree");
    MissionBuildHandlerBase::buildShutDown(temp, trees);
    model::SequenceWithMemory seq = sub_tree.removeChildren().insertNode<model::SequenceWithMemory>();
    seq.insertNode<model::TerminateExecutor>().set_executor(EVENT_MONITOR_EXECUTOR_NAME);
    seq.insertTree(temp);
  }
};

}  // namespace auto_apms_mission

AUTO_APMS_BEHAVIOR_TREE_DECLARE_BUILD_HANDLER(auto_apms_mission::SingleNodeMissionBuildHandler)
AUTO_APMS_BEHAVIOR_TREE_DECLARE_BUILD_HANDLER(auto_apms_mission::MultipleNodesMissionBuildHandler)