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

class MissionBuildHandler : public MissionBuildHandlerBase
{
public:
  using MissionBuildHandlerBase::MissionBuildHandlerBase;

private:
  void buildBringUp(model::SequenceWithMemory & sequence, const std::vector<TreeResource::Identity> & trees) override
  {
    for (const TreeResource::Identity & r : trees) {
      sequence.insertTreeFromResource(r);
    }
  }

  void buildMission(model::SequenceWithMemory & sequence, const std::vector<TreeResource::Identity> & trees) override
  {
    // TreeDocument doc;
    // TreeDocument::TreeElement mission_tree = doc.newTree("Mission");
    // TreeDocument::NodeElement mission_sequence = mission_tree.insertNode<model::SequenceWithMemory>();
    // for (const TreeResource::Identity & r : trees) {
    //   mission_sequence.insertTreeFromResource(r);
    // }
    // auto_apms_behavior_tree::insertStartExecutorFromString(sequence, mission_tree)
    //   .set_attach(true)
    //   .set_clear_blackboard(true)
    //   .set_executor(MISSION_EXECUTOR_NAME);

    for (const TreeResource::Identity & r : trees) {
      sequence.insertTreeFromResource(r);
    }
  }

  void buildEventMonitor(
    TreeDocument::TreeElement & sub_tree,
    const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & contingencies,
    const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & emergencies) override
  {
    std::vector<TreeResource::Identity> event_ids;

    // Emergencies have higher priority than contingencies (they are inserted to the vector first)
    for (const auto & [event_id, handler_id] : emergencies) {
      event_ids.push_back(event_id);
    }
    for (const auto & [event_id, handler_id] : contingencies) {
      if (auto_apms_util::contains(event_ids, event_id)) event_ids.push_back(event_id);
    }

    model::Fallback fallback = sub_tree.insertNode<model::Fallback>();
    for (const TreeResource::Identity & r : event_ids) {
      fallback.insertTreeFromResource(r).setConditionalScript(BT::PostCond::ON_SUCCESS, "event_id = '" + r.str() + "'");
    }
  }

  void buildContingencyHandling(
    TreeDocument::TreeElement & sub_tree,
    const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & contingencies) override
  {
    model::ReactiveFallback fallback = sub_tree.insertNode<model::ReactiveFallback>();
    for (const auto & [event_id, handler_id] : contingencies) {
      model::AsyncSequence seq = fallback.insertNode<model::AsyncSequence>();
      seq.insertNode<model::ScriptCondition>().set_code("event_id == '" + event_id.str() + "'");
      seq.insertTreeFromResource(handler_id);
    }
  }

  void buildEmergencyHandling(
    TreeDocument::TreeElement & sub_tree,
    const std::vector<std::pair<TreeResource::Identity, TreeResource::Identity>> & emergencies) override
  {
    model::ReactiveFallback fallback = sub_tree.insertNode<model::ReactiveFallback>();
    for (const auto & [event_id, handler_id] : emergencies) {
      model::AsyncSequence seq = fallback.insertNode<model::AsyncSequence>();
      seq.insertNode<model::ScriptCondition>().set_code("event_id == '" + event_id.str() + "'");
      seq.insertTreeFromResource(handler_id);
    }
  }

  void buildShutDown(model::SequenceWithMemory & sequence, const std::vector<TreeResource::Identity> & trees) override
  {
    for (const TreeResource::Identity & r : trees) {
      sequence.insertTreeFromResource(r);
    }
  }
};

}  // namespace auto_apms_mission

AUTO_APMS_BEHAVIOR_TREE_DECLARE_BUILD_HANDLER(auto_apms_mission::MissionBuildHandler)