// Copyright 2024 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "auto_apms_mission/mission_config.hpp"

#include <filesystem>

#include "ament_index_cpp/get_resource.hpp"
#include "auto_apms_util/exceptions.hpp"
#include "auto_apms_util/resource.hpp"
#include "auto_apms_util/string.hpp"

namespace auto_apms_mission
{

MissionConfigResourceIdentity::MissionConfigResourceIdentity(const std::string & identity)
: BehaviorResourceIdentity(identity, _AUTO_APMS_MISSION__DEFAULT_BEHAVIOR_CATEGORY__MISSION)
{
}

MissionConfigResourceIdentity::MissionConfigResourceIdentity(const char * identity)
: MissionConfigResourceIdentity(std::string(identity))
{
}

const std::string MissionConfig::YAML_KEY_BRINGUP = "BRINGUP";
const std::string MissionConfig::YAML_KEY_MISSION = "MISSION";
const std::string MissionConfig::YAML_KEY_CONTINGENCY = "CONTINGENCY";
const std::string MissionConfig::YAML_KEY_EMERGENCY = "EMERGENCY";
const std::string MissionConfig::YAML_KEY_SHUTDOWN = "SHUTDOWN";

MissionConfig MissionConfig::fromResource(const MissionConfigResourceIdentity & identity)
{
  MissionConfigResource resource(identity);
  return fromFile(resource.build_request_file_path_);
}

}  // namespace auto_apms_mission