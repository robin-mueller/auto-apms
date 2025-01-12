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

/// @cond INTERNAL
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
 * @brief Data structure for information on which behavior tree node plugin to load and how to configure them.
 */
class NodeManifest
{
public:
  using RegistrationOptions = NodeRegistrationOptions;

  /// Mapping of a node's name and its registration parameters.
  using Map = std::map<std::string, NodeRegistrationOptions>;

  /**
   * @brief Constructor of a NodeManifest data structure.
   *
   * The manifest is empty initially, but you may provide a map to initialize it.
   * @param map Initial manifest data.
   */
  NodeManifest(const Map & map = {});

  AUTO_APMS_UTIL_DEFINE_YAML_CONVERSION_METHODS(NodeManifest)

  /**
   * @brief Create a node plugin manifest from multiple files. They are loaded in the given order.
   * @param paths Paths to the manifest files.
   * @throw auto_apms_behavior_tree::exceptions::NodeManifestError if NodeManifest::merge fails for any file.
   */
  static NodeManifest fromFiles(const std::vector<std::string> & paths);

  /**
   * @brief Create a node manifest from an installed resource.
   *
   * The resource identity must be specified in the format `<package_name>::<file_stem>` or simply `<file_stem>`.
   * @param identity Identity of the node manifest resource.
   * @return Node manifest created from the corresponding resource.
   * @throw auto_apms_util::exceptions::ResourceIdentityFormatError if @p identity has wrong format.
   * @throw auto_apms_util::exceptions::ResourceError if resource cannot be determined using @p identity.
   */
  static NodeManifest fromResourceIdentity(const std::string & identity);

  /**
   * @brief Write the node manifest to a file.
   * @param file_path Path to the target file.
   * @throw auto_apms_behavior_tree::exceptions::NodeManifestError if file cannot be opened.
   */
  void toFile(const std::string & file_path) const;

  /**
   * @brief Determine if a behavior tree node has been added to the manifest.
   * @param node_name Name of the behavior tree node.
   * @return `true` if name is existing, `false` otherwise.
   */
  bool contains(const std::string & node_name) const;

  /**
   * @brief Access the node manifest and retrieve registration options for a specific behavior tree node.
   * @param node_name Name of the behavior tree node.
   * @return Registration options for @p node_name.
   * @throw std::out_of_range if @p node_name doesn't exist.
   */
  RegistrationOptions & operator[](const std::string & node_name);

  /// @copydoc NodeManifest::operator[]
  const RegistrationOptions & operator[](const std::string & node_name) const;

  /**
   * @brief Add registration options for a behavior tree node to the manifest.
   * @param node_name Name of the behavior tree node.
   * @param opt Registration options to be used when loading the behavior tree node.
   * @return Modified node manifest.
   * @throw auto_apms_behavior_tree::exceptions::NodeManifestError if the registration options are invalid or
   * `node_name` already exists.
   */
  NodeManifest & add(const std::string & node_name, const RegistrationOptions & opt);

  /**
   * @brief Remove registration options for a behavior tree node.
   * @param node_name Name of the behavior tree node.
   * @return Modified node manifest.
   * @throw std::out_of_range if @p node_name doesn't exist.
   */
  NodeManifest & remove(const std::string & node_name);

  /**
   * @brief Merges another NodeManifest with this one.
   * @param other Other node manifest.
   * @param replace `true` for automatically replacing entries with the same key. Throws an error if `false`
   * and `other` contains any keys that already exist in this manifest.
   * @return Modified node manifest.
   * @throw auto_apms_behavior_tree::exceptions::NodeManifestError if @p other shares entries and @p replace is `false`.
   */
  NodeManifest & merge(const NodeManifest & other, bool replace = false);

  /**
   * @brief Get all names of the behavior tree nodes specified by the manifest.
   * @return Vector of all available node names.
   */
  std::vector<std::string> getNodeNames();

  /**
   * @brief Get the number of behavior tree nodes this manifest holds registration options for.
   * @return Size of this node manifest.
   */
  size_t size() const;

  /**
   * @brief Determine whether any node registration options have been added to the manifest.
   * @return `true` if the node manifest is empty, `false` otherwise.
   */
  bool empty() const;

  /**
   * @brief Get a view of the internal map.
   * @return Internal map of the node manifest.
   */
  const Map & map() const;

private:
  Map map_;
};

}  // namespace auto_apms_behavior_tree::core

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

/// @cond INTERNAL
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
