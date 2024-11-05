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

#include <vector>
#include <map>

#include "auto_apms_util/yaml.hpp"
#include "auto_apms_behavior_tree/node/node_registration_params.hpp"

namespace auto_apms_behavior_tree
{
class NodeManifest;
}

/// @cond
namespace YAML
{
template <>
struct convert<auto_apms_behavior_tree::NodeManifest>
{
  using Manifest = auto_apms_behavior_tree::NodeManifest;
  static Node encode(const Manifest& rhs);
  static bool decode(const Node& node, Manifest& lhs);
};
}  // namespace YAML
/// @endcond

namespace auto_apms_behavior_tree
{

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Data structure for resource lookup data and configuration parameters required for loading and
 * registering multiple behavior tree node plugins.
 */
class NodeManifest
{
public:
  using Params = NodeRegistrationParams;

  /// Mapping of a node's name and its registration parameters.
  using ParamMap = std::map<std::string, NodeRegistrationParams>;

  NodeManifest(const ParamMap& param_map = {});

  AUTO_APMS_DEFINE_YAML_INTERPRETER_METHODS(NodeManifest)

  /**
   * @brief Create a node plugin manifest from a file.
   * @param file_path Path to the manifest file.
   */
  static NodeManifest fromFile(const std::string& file_path);

  /**
   * @brief Create a node plugin manifest from multiple files. They are loaded in the given order.
   * @param file_paths Paths to the manifest files.
   */
  static NodeManifest fromFiles(const std::vector<std::string>& file_paths);

  void toFile(const std::string& file_path) const;

  bool contains(const std::string& node_name) const;

  Params& operator[](const std::string& node_name);
  const Params& operator[](const std::string& node_name) const;

  NodeManifest& add(const std::string& node_name, const Params& p);

  NodeManifest& remove(const std::string& node_name);

  NodeManifest& merge(const NodeManifest& m);

  const ParamMap& getInternalMap() const;

private:
  ParamMap param_map_;
};

}  // namespace auto_apms_behavior_tree

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

/// @cond
namespace YAML
{
inline Node convert<auto_apms_behavior_tree::NodeManifest>::encode(const Manifest& rhs)
{
  Node node;
  for (const auto& [name, params] : rhs.getInternalMap())
    node[name] = params;
  return node;
}
inline bool convert<auto_apms_behavior_tree::NodeManifest>::decode(const Node& node, Manifest& lhs)
{
  if (!node.IsMap())
    throw std::runtime_error("YAML::Node for auto_apms_behavior_tree::NodeManifest must be map but is type " +
                             std::to_string(node.Type()) +
                             " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");

  for (auto it = node.begin(); it != node.end(); ++it)
  {
    const auto& name = it->first.as<std::string>();
    lhs.add(name, it->second.as<Manifest::Params>());
  }
  return true;
}
}  // namespace YAML
/// @endcond
