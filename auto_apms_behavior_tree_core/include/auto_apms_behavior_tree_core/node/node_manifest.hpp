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

#include "auto_apms_behavior_tree_core/node/node_registration_options.hpp"
#include "auto_apms_util/exceptions.hpp"
#include "auto_apms_util/yaml.hpp"

namespace auto_apms_behavior_tree::core
{
class NodeManifest;
}

/// @cond
namespace YAML
{
template <>
struct convert<auto_apms_behavior_tree::core::NodeManifest>
{
  using Manifest = auto_apms_behavior_tree::core::NodeManifest;
  static Node encode(const Manifest & rhs);
  static bool decode(const Node & node, Manifest & lhs);
};
}  // namespace YAML
/// @endcond

namespace auto_apms_behavior_tree::core
{

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Data structure for resource lookup data and configuration parameters required for loading and
 * registering multiple behavior tree node plugins.
 */
class NodeManifest
{
public:
  using RegistrationOptions = NodeRegistrationOptions;

  /// Mapping of a node's name and its registration parameters.
  using Map = std::map<std::string, NodeRegistrationOptions>;

  NodeManifest(const Map & map = {});

  AUTO_APMS_UTIL_DEFINE_YAML_CONVERSION_METHODS(NodeManifest)

  /**
   * @brief Create a node plugin manifest from multiple files. They are loaded in the given order.
   * @param file_paths Paths to the manifest files.
   */
  static NodeManifest fromFiles(const std::vector<std::string> & paths);

  static NodeManifest fromResourceIdentity(const std::string & identity);

  void toFile(const std::string & file_path) const;

  bool contains(const std::string & node_name) const;

  RegistrationOptions & operator[](const std::string & node_name);
  const RegistrationOptions & operator[](const std::string & node_name) const;

  NodeManifest & add(const std::string & node_name, const RegistrationOptions & p);

  NodeManifest & remove(const std::string & node_name);

  NodeManifest & merge(const NodeManifest & m);

  std::vector<std::string> getNodeNames();

  size_t size() const;

  bool empty() const;

  const Map & map() const;

private:
  Map map_;
};

}  // namespace auto_apms_behavior_tree::core

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

/// @cond
namespace YAML
{
inline Node convert<auto_apms_behavior_tree::core::NodeManifest>::encode(const Manifest & rhs)
{
  Node node(NodeType::Map);
  for (const auto & [name, params] : rhs.map()) node[name] = params;
  return node;
}
inline bool convert<auto_apms_behavior_tree::core::NodeManifest>::decode(const Node & node, Manifest & rhs)
{
  if (!node.IsMap())
    throw auto_apms_util::exceptions::YAMLFormatError(
      "YAML::Node for auto_apms_behavior_tree::core::NodeManifest must be map but is type " +
      std::to_string(node.Type()) + " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");

  for (auto it = node.begin(); it != node.end(); ++it) {
    const auto & name = it->first.as<std::string>();
    try {
      rhs.add(name, it->second.as<Manifest::RegistrationOptions>());
    } catch (const std::exception & e) {
      throw auto_apms_util::exceptions::YAMLFormatError(
        "Node registration parameters for node '" + name + "' are invalid: " + e.what());
    }
  }
  return true;
}
}  // namespace YAML
/// @endcond
