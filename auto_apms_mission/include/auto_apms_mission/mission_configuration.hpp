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

#pragma once

#include <map>
#include <vector>

#include "auto_apms_util/exceptions.hpp"
#include "auto_apms_util/yaml.hpp"

namespace auto_apms_mission
{
struct MissionConfiguration;
}

/// @cond
namespace YAML
{
template <>
struct convert<auto_apms_mission::MissionConfiguration>
{
  using Config = auto_apms_mission::MissionConfiguration;
  static Node encode(const Config & rhs);
  static bool decode(const Node & node, Config & lhs);
};
}  // namespace YAML
/// @endcond

namespace auto_apms_mission
{

struct MissionConfiguration
{
  static const std::string YAML_KEY_BRINGUP;
  static const std::string YAML_KEY_MISSION;
  static const std::string YAML_KEY_CONTINGENCY;
  static const std::string YAML_KEY_SHUTDOWN;

  MissionConfiguration() = default;

  AUTO_APMS_UTIL_DEFINE_YAML_CONVERSION_METHODS(MissionConfiguration)

  static MissionConfiguration fromResourceIdentity(const std::string identity);

  std::vector<std::string> bringup;
  std::vector<std::string> mission;
  std::map<std::string, std::vector<std::string>> contingency;
  std::vector<std::string> shutdown;
};

}  // namespace auto_apms_mission

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

/// @cond
namespace YAML
{
inline Node convert<auto_apms_mission::MissionConfiguration>::encode(const Config & rhs)
{
  Node node(NodeType::Map);
  node[Config::YAML_KEY_BRINGUP] = rhs.bringup;
  node[Config::YAML_KEY_MISSION] = rhs.mission;
  node[Config::YAML_KEY_CONTINGENCY] = rhs.contingency;
  node[Config::YAML_KEY_SHUTDOWN] = rhs.shutdown;
  return node;
}
inline bool convert<auto_apms_mission::MissionConfiguration>::decode(const Node & node, Config & rhs)
{
  if (!node.IsMap())
    throw auto_apms_util::exceptions::YAMLFormatError(
      "YAML::Node for auto_apms_mission::MissionConfiguration must be map but is type " + std::to_string(node.Type()) +
      " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");

  for (auto it = node.begin(); it != node.end(); ++it) {
    const std::string key = it->first.as<std::string>();
    const Node & val = it->second;

    if (key == Config::YAML_KEY_BRINGUP) {
      if (val.IsNull()) continue;
      if (!val.IsSequence()) {
        throw auto_apms_util::exceptions::YAMLFormatError(
          "Value for key '" + key + "' must be a sequence but is type " + std::to_string(val.Type()) +
          " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
      }
      rhs.bringup = val.as<std::vector<std::string>>();
      continue;
    }

    if (key == Config::YAML_KEY_MISSION) {
      if (!val.IsSequence()) {
        throw auto_apms_util::exceptions::YAMLFormatError(
          "Value for key '" + key + "' must be a sequence but is type " + std::to_string(val.Type()) +
          " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
      }
      rhs.mission = val.as<std::vector<std::string>>();
      continue;
    }

    if (key == Config::YAML_KEY_CONTINGENCY) {
      if (val.IsNull()) continue;
      if (!val.IsMap()) {
        throw auto_apms_util::exceptions::YAMLFormatError(
          "Value for key '" + key + "' must be a map but is type " + std::to_string(val.Type()) +
          " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
      }
      rhs.contingency = val.as<std::map<std::string, std::vector<std::string>>>();
      continue;
    }

    if (key == Config::YAML_KEY_SHUTDOWN) {
      if (val.IsNull()) continue;
      if (!val.IsSequence()) {
        throw auto_apms_util::exceptions::YAMLFormatError(
          "Value for key '" + key + "' must be a sequence but is type " + std::to_string(val.Type()) +
          " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
      }
      rhs.shutdown = val.as<std::vector<std::string>>();
      continue;
    }

    // Unkown parameter
    throw auto_apms_util::exceptions::YAMLFormatError("Unkown parameter name '" + key + "'.");
  }
  return true;
}
}  // namespace YAML
/// @endcond