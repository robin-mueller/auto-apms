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

#include "auto_apms_behavior_tree/node/node_registration_params.hpp"
#include "auto_apms_behavior_tree/resource/node_class_loader.hpp"

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

  static const std::string PARAM_NAME_CLASS;
  static const std::string PARAM_NAME_PACKAGE;
  static const std::string PARAM_NAME_LIBRARY;
  static const std::string PARAM_NAME_PORT;
  static const std::string PARAM_NAME_WAIT_TIMEOUT;
  static const std::string PARAM_NAME_REQUEST_TIMEOUT;

  NodeManifest(const ParamMap& param_map = {});

  /**
   * @brief Create a node plugin manifest from multiple files. They are loaded in the given order.
   * @param file_paths Paths to the manifest files.
   */
  static NodeManifest fromFiles(const std::vector<std::string>& file_paths);

  /**
   * @brief Create a node plugin manifest from a file.
   * @param file_path Path to the manifest file.
   */
  static NodeManifest fromFile(const std::string& file_path);

  static NodeManifest fromString(const std::string& manifest_str);

  bool contains(const std::string& node_name) const;

  NodeRegistrationParams& operator[](const std::string& node_name);
  const NodeRegistrationParams& operator[](const std::string& node_name) const;

  NodeManifest& add(const std::string& node_name, const NodeRegistrationParams& p);

  NodeManifest& remove(const std::string& node_name);

  NodeManifest& merge(const NodeManifest& m);

  /**
   * @brief Automatically fill node plugin resource information in @p manifest.
   *
   * This function autocompletes the package and library fields of the manifest if possible. Depending on the value of
   * the package field, the manifest will be updated:
   *
   * - **Package undefined**: The library path will be resolved by looking up the class name
   * in @p class_loader.
   *
   * - **Package defined**: The library path will be resolved by looking up the class name
   * in @p class_loader considering only resources registered by the given package.
   *
   * The library parameter will be overwritten in any case. Other parameters remain the same and are simply copied.
   *
   * This method is used internally. Mostly for introspection purposes.
   *
   * @param class_loader pluginlib::ClassLoader object managing behavior tree node plugin resources.
   * @return Reference to the altered manifest object.
   * @throw auto_apms_util::exceptions::ResourceNotFoundError if no unique resource for a node can be found, thus the
   * library path cannot be filled automatically.
   */
  NodeManifest& autoComplete(NodePluginClassLoader& class_loader);

  void toFile(const std::string& file_path) const;

  std::string toString() const;

  const ParamMap& getInternalMap() const;

private:
  ParamMap param_map_;
};

}  // namespace auto_apms_behavior_tree
