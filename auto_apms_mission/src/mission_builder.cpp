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

namespace auto_apms_mission
{

class MissionBuildHandler : public MissionBuildHandlerBase
{
public:
  using MissionBuildHandlerBase::MissionBuildHandlerBase;

private:
  void buildBringUp(model::SequenceWithMemory & sequence, const std::vector<TreeResource> & trees) override final
  {
    for (const TreeResource & r : trees) {
      sequence.insertTreeFromResource(r);
    }
  }

  void buildMission(model::SequenceWithMemory & sequence, const std::vector<TreeResource> & trees) override final
  {
    TreeDocument doc;
    TreeDocument::TreeElement mission_tree = doc.newTree("Mission");
    TreeDocument::NodeElement mission_sequence = mission_tree.insertNode<model::SequenceWithMemory>();
    for (const TreeResource & r : trees) {
      mission_sequence.insertTreeFromResource(r);
    }
    auto_apms_behavior_tree::insertStartExecutorFromString(sequence, mission_tree)
      .set_attach(true)
      .set_clear_blackboard(true)
      .set_executor(MISSION_EXECUTOR_NAME);
  }

  void buildShutDown(model::SequenceWithMemory & sequence, const std::vector<TreeResource> & trees) override final
  {
    for (const TreeResource & r : trees) {
      sequence.insertTreeFromResource(r);
    }
  }
};

}  // namespace auto_apms_mission

AUTO_APMS_BEHAVIOR_TREE_DECLARE_BUILD_HANDLER(auto_apms_mission::MissionBuildHandler)