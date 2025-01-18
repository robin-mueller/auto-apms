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

#include <utility>
#include <vector>

#include "auto_apms_behavior_tree_core/tree/tree_resource.hpp"
#include "auto_apms_util/exceptions.hpp"
#include "auto_apms_util/yaml.hpp"

namespace auto_apms_mission
{
struct MissionConfiguration;
}

/// @cond INTERNAL
namespace YAML
{
template <>
struct convert<auto_apms_mission::MissionConfiguration>
{
  using Config = auto_apms_mission::MissionConfiguration;
  static Node encode(const Config & rhs);
  static bool decode(const Node & node, Config & rhs);
};
}  // namespace YAML
/// @endcond

namespace auto_apms_mission
{

/**
 * @ingroup auto_apms_mission
 * @brief Configuration parameters for standard mission supported by AutoAPMS.
 */
struct MissionConfiguration
{
  using TreeResourceIdentity = auto_apms_behavior_tree::core::TreeResourceIdentity;

  static const std::string YAML_KEY_BRINGUP;
  static const std::string YAML_KEY_MISSION;
  static const std::string YAML_KEY_CONTINGENCY;
  static const std::string YAML_KEY_EMERGENCY;
  static const std::string YAML_KEY_SHUTDOWN;

  MissionConfiguration() = default;

  AUTO_APMS_UTIL_DEFINE_YAML_CONVERSION_METHODS(MissionConfiguration)

  static MissionConfiguration fromResourceIdentity(const std::string identity);

  std::vector<TreeResourceIdentity> bringup;
  std::vector<TreeResourceIdentity> mission;
  std::vector<std::pair<TreeResourceIdentity, TreeResourceIdentity>> contingency;
  std::vector<std::pair<TreeResourceIdentity, TreeResourceIdentity>> emergency;
  std::vector<TreeResourceIdentity> shutdown;
};

}  // namespace auto_apms_mission

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

/// @cond INTERNAL
namespace YAML
{
inline Node convert<auto_apms_mission::MissionConfiguration>::encode(const Config & rhs)
{
  Node node(NodeType::Map);
  node[Config::YAML_KEY_BRINGUP] = rhs.bringup;
  node[Config::YAML_KEY_MISSION] = rhs.mission;
  Node contingency_node = node[Config::YAML_KEY_CONTINGENCY];
  for (const auto & [key, val] : rhs.contingency) {
    contingency_node[key.str()] = val;
  }
  Node emergency_node = node[Config::YAML_KEY_EMERGENCY];
  for (const auto & [key, val] : rhs.emergency) {
    emergency_node[key.str()] = val;
  }
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
    const Node val = it->second;

    if (key == Config::YAML_KEY_BRINGUP) {
      if (val.IsNull()) continue;
      if (!val.IsSequence()) {
        throw auto_apms_util::exceptions::YAMLFormatError(
          "Value for key '" + key + "' must be a sequence but is type " + std::to_string(val.Type()) +
          " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
      }
      rhs.bringup = val.as<std::vector<Config::TreeResourceIdentity>>();
      continue;
    }

    if (key == Config::YAML_KEY_MISSION) {
      if (!val.IsSequence()) {
        throw auto_apms_util::exceptions::YAMLFormatError(
          "Value for key '" + key + "' must be a sequence but is type " + std::to_string(val.Type()) +
          " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
      }
      rhs.mission = val.as<std::vector<Config::TreeResourceIdentity>>();
      continue;
    }

    if (key == Config::YAML_KEY_CONTINGENCY) {
      if (val.IsNull()) continue;
      if (!val.IsMap()) {
        throw auto_apms_util::exceptions::YAMLFormatError(
          "Value for key '" + key + "' must be a map but is type " + std::to_string(val.Type()) +
          " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
      }
      for (YAML::const_iterator it2 = val.begin(); it2 != val.end(); ++it2) {
        const std::string monitor_id = it2->first.as<std::string>();
        if (!it2->second.IsScalar()) {
          throw auto_apms_util::exceptions::YAMLFormatError(
            "Value for key '" + monitor_id + "' in the CONTINGENCY group must be scalar but is type " +
            std::to_string(val.Type()) + " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
        }
        rhs.contingency.push_back(
          {Config::TreeResourceIdentity(monitor_id), it2->second.as<Config::TreeResourceIdentity>()});
      }
      continue;
    }

    if (key == Config::YAML_KEY_EMERGENCY) {
      if (val.IsNull()) continue;
      if (!val.IsMap()) {
        throw auto_apms_util::exceptions::YAMLFormatError(
          "Value for key '" + key + "' must be a map but is type " + std::to_string(val.Type()) +
          " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
      }
      for (YAML::const_iterator it2 = val.begin(); it2 != val.end(); ++it2) {
        const std::string monitor_id = it2->first.as<std::string>();
        if (!it2->second.IsScalar()) {
          throw auto_apms_util::exceptions::YAMLFormatError(
            "Value for key '" + monitor_id + "' in the EMERGENCY group must be scalar but is type " +
            std::to_string(val.Type()) + " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
        }
        rhs.emergency.push_back(
          {Config::TreeResourceIdentity(monitor_id), it2->second.as<Config::TreeResourceIdentity>()});
      }
      continue;
    }

    if (key == Config::YAML_KEY_SHUTDOWN) {
      if (val.IsNull()) continue;
      if (!val.IsSequence()) {
        throw auto_apms_util::exceptions::YAMLFormatError(
          "Value for key '" + key + "' must be a sequence but is type " + std::to_string(val.Type()) +
          " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
      }
      rhs.shutdown = val.as<std::vector<Config::TreeResourceIdentity>>();
      continue;
    }

    // Unkown parameter
    throw auto_apms_util::exceptions::YAMLFormatError("Unkown parameter name '" + key + "'.");
  }
  return true;
}
}  // namespace YAML
/// @endcond
