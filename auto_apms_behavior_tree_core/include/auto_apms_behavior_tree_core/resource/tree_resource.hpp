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

#include <set>
#include <string>
#include <vector>

namespace auto_apms_behavior_tree::core
{

/**
 * @brief Struct containing behavior tree resource data
 * @ingroup auto_apms_behavior_tree
 */
struct TreeResource
{
private:
  // The default constructor is private. To create an instance, use one of the static factory methods instead.
  TreeResource() = default;

public:
  std::string tree_file_stem;
  std::string tree_file_path;
  std::string package_name;
  std::string node_manifest_file_path;
  std::set<std::string> tree_names;

  /**
   * @brief Collect all behavior tree resources registered by a certain package.
   * @param package_name Name of the package to search for resources.
   * @return Collection of all resources found in @p package_name.
   */
  static std::vector<TreeResource> collectFromPackage(const std::string & package_name);

  static TreeResource selectByTreeName(const std::string & tree_name, const std::string & package_name = "");

  static TreeResource selectByFileName(const std::string & file_name, const std::string & package_name = "");

  /**
   * @brief Find a behavior tree resource using an identity string.
   *
   * To uniquely identify the resource, the @p identity string may contain the
   * `<tree_file_stem>`, the `<tree_name>` and the `<package_name>` seperated by `::` in that order. Depending on the
   * registered resources, it might be possible to use shorter, less precise signatures. All possible identity strings
   * are listed below:
   *
   * - `<tree_file_stem>::<tree_name>::<package_name>`
   *
   *   Fully qualified identity string of a behavior tree resource.
   *
   * - `<tree_file_stem>::<tree_name>::`
   *
   *   Try to find the resource by searching for a tree with name `<tree_name>` in a file with stem `<tree_file_stem>`
   * considering all packages.
   *
   * - `<tree_file_stem>::::`
   *
   *   Try to find the resource by searching for a file with stem `<tree_file_stem>` considering all packages.
   *
   * - `::<tree_name>::`
   *
   *   Try to find the resource by searching for a tree with name `<tree_name>` considering all packages.
   *
   * @note The delimiter `::` must be kept when tokens are omitted.
   *
   * @param identity Identity string with formatting compliant to the signatures above.
   * @return Corresponding TreeResource object.
   * @throws exceptions::ResourceIdentityFormatError if the identity string has wrong format.
   * @throws auto_apms_util::exceptions::ResourceError if the resource cannot be found using the given
   * identity string.
   */
  static TreeResource fromString(const std::string & identity);

  std::string writeTreeToString() const;
};

}  // namespace auto_apms_behavior_tree::core