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

class MultiNodeMissionBuildHandler : public MissionBuildHandlerBase
{
public:
  MultiNodeMissionBuildHandler(rclcpp::Node::SharedPtr ros_node_ptr, NodeLoader::SharedPtr tree_node_loader_ptr)
  : MissionBuildHandlerBase("multi_node_mission", ros_node_ptr, tree_node_loader_ptr)
  {
  }

private:
  MissionConfig createMissionConfig(const std::string & build_request) override final
  {
    return MissionConfig::fromResource(build_request);
  }

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
      .setConditionalScript(BT::PostCond::ON_SUCCESS, "@event_id = event_id")
      .setConditionalScript(BT::PostCond::ON_FAILURE, "@event_id = ''");
    TreeDocument::TreeElement main_tree = doc.newTree("InfiniteLoopEventMonitor");
    main_tree.insertNode<model::KeepRunningUntilFailure>().insertNode<model::ForceSuccess>().insertTree(
      event_monitor_tree);

    // Retrieve the event ID from the event monitor executor
    model::Sequence seq = sub_tree.removeChildren().insertNode<model::Sequence>();
    model::Sequence run_once_seq = seq.insertNode<model::RunOnce>().insertNode<model::Sequence>();
    run_once_seq.insertNode<model::SetParameterString>()
      .set_node(EVENT_MONITOR_EXECUTOR_NAME)
      .set_parameter("bb.event_id")
      .set_value("");
    auto_apms_behavior_tree::insertStartExecutorFromString(run_once_seq, main_tree)
      .setName("StartDetachedEventMonitor")
      .set_attach(false)
      .set_clear_blackboard(false)
      .set_executor(EVENT_MONITOR_EXECUTOR_NAME);
    seq.insertNode<model::GetParameterString>()
      .set_node(EVENT_MONITOR_EXECUTOR_NAME)
      .set_parameter("bb.event_id")
      .set_value("{event_id}");
  }

  void buildContingencyHandling(
    TreeDocument::TreeElement & sub_tree,
    const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & contingencies) override final
  {
    TreeDocument doc;
    TreeDocument::TreeElement handler_tree = doc.newTree(sub_tree.getName());
    model::Parallel parallel = handler_tree.insertNode<model::Parallel>().set_success_count(1).set_failure_count(1);
    parallel.insertNode<model::KeepRunningUntilFailure>()
      .insertNode<model::GetParameterString>()
      .set_node(EVENT_MONITOR_EXECUTOR_NAME)
      .set_parameter("bb.event_id")
      .set_value("{event_id}");
    TreeDocument::TreeElement base_tree = doc.newTree("HandlerBaseTree");
    MissionBuildHandlerBase::buildContingencyHandling(base_tree, contingencies);
    parallel.insertNode<model::WaitValueUpdate>().set_entry("{event_id}").insertTree(base_tree);

    auto_apms_behavior_tree::insertStartExecutorFromString(sub_tree.removeChildren(), handler_tree)
      .setName(handler_tree.getName())
      .set_attach(true)
      .set_clear_blackboard(true)
      .set_executor(EVENT_HANDLER_EXECUTOR_NAME);
  }

  void buildEmergencyHandling(
    TreeDocument::TreeElement & sub_tree,
    const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & emergencies) override final
  {
    TreeDocument doc;
    TreeDocument::TreeElement handler_tree = doc.newTree(sub_tree.getName());
    model::Parallel parallel = handler_tree.insertNode<model::Parallel>().set_success_count(1).set_failure_count(1);
    parallel.insertNode<model::KeepRunningUntilFailure>()
      .insertNode<model::GetParameterString>()
      .set_node(EVENT_MONITOR_EXECUTOR_NAME)
      .set_parameter("bb.event_id")
      .set_value("{event_id}");
    TreeDocument::TreeElement base_tree = doc.newTree("HandlerBaseTree");
    MissionBuildHandlerBase::buildEmergencyHandling(base_tree, emergencies);
    parallel.insertNode<model::WaitValueUpdate>().set_entry("{event_id}").insertTree(base_tree);

    auto_apms_behavior_tree::insertStartExecutorFromString(sub_tree.removeChildren(), handler_tree)
      .setName(handler_tree.getName())
      .set_attach(true)
      .set_clear_blackboard(true)
      .set_executor(EVENT_HANDLER_EXECUTOR_NAME);
  }

  void buildShutDown(TreeDocument::TreeElement & sub_tree, const std::vector<TreeResource::Identity> & trees)
  {
    // First thing: Terminate detached event monitor executor
    model::SequenceWithMemory seq = sub_tree.removeChildren().insertNode<model::SequenceWithMemory>();
    seq.insertNode<model::TerminateExecutor>().set_executor(EVENT_MONITOR_EXECUTOR_NAME);

    // Afterwards, execute everything else
    TreeDocument doc;
    TreeDocument::TreeElement temp = doc.newTree("TempTree");
    MissionBuildHandlerBase::buildShutDown(temp, trees);
    seq.insertTree(temp);
  }
};

}  // namespace auto_apms_mission

AUTO_APMS_BEHAVIOR_TREE_DECLARE_BUILD_HANDLER(auto_apms_mission::MultiNodeMissionBuildHandler)