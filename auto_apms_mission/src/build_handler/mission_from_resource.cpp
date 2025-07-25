// Copyright 2025 Robin Müller
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

/**
 * @brief Standard build handler for building a mission from an installed resource.
 *
 * This build handler accepts a single mission config resource identity string as a build request.
 *
 * @sa MissionConfigResource
 */
class MissionFromResourceBuildHandler : public MissionBuildHandlerBase
{
public:
  MissionFromResourceBuildHandler(rclcpp::Node::SharedPtr ros_node_ptr, NodeLoader::SharedPtr tree_node_loader_ptr)
  : MissionBuildHandlerBase("mission_from_resource", ros_node_ptr, tree_node_loader_ptr)
  {
  }

  MissionConfig createMissionConfig(const std::string & build_request) override final
  {
    return MissionConfig::fromResource(build_request);
  }
};

}  // namespace auto_apms_mission

AUTO_APMS_BEHAVIOR_TREE_DECLARE_BUILD_HANDLER(auto_apms_mission::MissionFromResourceBuildHandler)