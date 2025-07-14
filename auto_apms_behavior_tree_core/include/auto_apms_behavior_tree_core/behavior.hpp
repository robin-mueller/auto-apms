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

#pragma once

#include <set>
#include <string>
#include <vector>

#include "auto_apms_util/yaml.hpp"

namespace auto_apms_behavior_tree::core
{

/// Delimiter used to separate the category name from the rest.
static const std::string BEHAVIOR_RESOURCE_IDENTITY_CATEGORY_SEPARATOR = "/";

/// Delimiter used to separate the package name and resource name.
static const std::string BEHAVIOR_RESOURCE_IDENTITY_RESOURCE_SEPARATOR = "::";

/**
 * @brief Struct that encapsulates the identity string for a registered behavior.
 */
struct BehaviorResourceIdentity
{
  /**
   * @brief Constructor of a behavior resource identity object.
   *
   * @p identity must be formatted like `<category_name>/<package_name>::<resource_name>`.
   * @param identity Identity string for a specific behavior resource.
   * @throws auto_apms_util::exceptions::ResourceIdentityFormatError if the identity string has wrong format.
   */
  BehaviorResourceIdentity(const std::string & identity);

  /**
   * @brief Constructor of a behavior resource identity object.
   *
   * @p identity must be formatted like `<category_name>/<package_name>::<resource_name>`.
   * @param identity C-style identity string for a specific behavior resource.
   * @throws auto_apms_util::exceptions::ResourceIdentityFormatError if the identity string has wrong format.
   */
  BehaviorResourceIdentity(const char * identity);

  /**
   * @brief Constructor of an empty behavior resource identity object.
   *
   * The user must manually populate the member fields according to the behavior this object should identify.
   */
  BehaviorResourceIdentity() = default;

  virtual ~BehaviorResourceIdentity() = default;

  bool operator==(const BehaviorResourceIdentity & other) const;

  bool operator<(const BehaviorResourceIdentity & other) const;

  /**
   * @brief Create the corresponding identity string.
   * @return Identity string for a behavior resource.
   */
  std::string str() const;

  /**
   * @brief Determine whether this behavior resource identity object is considered empty.
   * @return `true` if none of the member fields are set, `false` otherwise.
   */
  bool empty() const;

  /// Name of the category this behavior resource belongs to.
  std::string category_name;
  /// Name of the package that registers the behavior resource.
  std::string package_name;
  /// Name or identifier for a single registered behavior.
  std::string resource_name;
};

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Class containing behavior resource data
 *
 * Behavior resources are registered by calling the CMake macro `auto_apms_behavior_tree_register_behaviors` in the
 * CMakeLists.txt of a package. They can be discovered once the corresponding package has been installed to the ROS 2
 * workspace.
 *
 * The user may refer to a specific resource using an identity string that may contain the tokens `<category_name>`,
 * `<package_name>`, and `<resource_name>`. Depending on the number of registered resources, it might be
 * convenient to use shorter, less precise signatures. The formatting is described below:
 *
 * - `<category_name>/<package_name>::<resource_name>`
 *
 *   Fully qualified identity string of a specific behavior tree.
 *
 * - `<package_name>::<resource_name>` or `/<package_name>::<resource_name>`
 *
 *   The category name is always optional. If omitted, the resource is searched in all categories.
 *
 * - `::<resource_name>` or `<resource_name>`
 *
 *   Just like the category name, the package name is optional as well. If omitted, the resource is searched in all
 *   packages.
 *
 * @note The token <resource_name> is usually the stem of the file given to
 * `auto_apms_behavior_tree_register_behaviors`. However, it is also possible to provide raw strings as arguments
 * (e.g. for referring to another resource). In this case, <resource_name> is determined to be the given string.
 *
 * ## Usage
 *
 * Given the user has specified a behavior using a YAML file, the CMake macro
 * `auto_apms_behavior_tree_register_behaviors` must be called in the CMakeLists.txt of the parent package (for example
 * `my_package`) like this:
 *
 * ```cmake
 * auto_apms_behavior_tree_register_behaviors(
 *     "config/my_behavior.yaml"
 * )
 * ```
 *
 * The macro automatically installs the given file and makes it discoverable using the above mentioned identities. In
 * the C++ source code, one may use this resource like this after the parent package `my_package` has been
 * installed:
 *
 * ```cpp
 * #include "auto_apms_behavior_tree_core/behavior.hpp"
 *
 * using namespace auto_apms_behavior_tree;
 *
 * // For example, use the fully qualified behavior resource identity signature
 * const std::string identity_string = "default/my_package::my_behavior";
 *
 * // You may use the proxy class for a tree resource identity
 * core::BehaviorResourceIdentity identity(identity_string);
 * core::BehaviorResource resource(identity);
 *
 * // Or instantiate the resource object directly from the corresponding identity string
 * core::BehaviorResource resource(identity_string);
 * ```
 *
 */
class BehaviorResource
{
public:
  using Identity = BehaviorResourceIdentity;

  /**
   * @brief Assemble a behavior tree resource using a TreeResourceIdentity.
   * @param identity Tree resource identity object.
   * @throws auto_apms_util::exceptions::ResourceError if the resource cannot be found using the given
   * identity.
   */
  TreeResource(const Identity & identity);

  /**
   * @brief Assemble a behavior tree resource identified by a string.
   *
   * @p identity must be formatted like `<package_name>::<tree_file_stem>::<tree_name>`.
   * @param identity Identity string for a specific behavior tree resource.
   * @throws auto_apms_util::exceptions::ResourceIdentityFormatError if the identity string has wrong format.
   * @throws auto_apms_util::exceptions::ResourceError if the resource cannot be found using the given identity
   string.
   */
  TreeResource(const std::string & identity);

  /**
   * @brief Assemble a behavior tree resource identified by a string.
   *
   * @p identity must be formatted like `<package_name>::<tree_file_stem>::<tree_name>`.
   * @param identity C-style identity string for a specific behavior tree resource.
   * @throws auto_apms_util::exceptions::ResourceIdentityFormatError if the identity string has wrong format.
   * @throws auto_apms_util::exceptions::ResourceError if the resource cannot be found using the given identity
   string.
   */
  TreeResource(const char * identity);

  /**
   * @brief Find an installed behavior tree resource using a specific behavior tree name.
   *
   * This factory method acts equivalently to passing an identity string formatted like
   `<package_name>::::<tree_name>`
   * to the constructor.
   * @param tree_name Name of the desired behavior tree.
   * @param package_name Optional package name provided to narrow down the search. If empty, search in all installed
   * packages.
   * @return Corresponding tree resource object.
   * @throws auto_apms_util::exceptions::ResourceError if the corresponding behavior tree cannot be found using the
   * given arguments.
   */
  static TreeResource selectByTreeName(const std::string & tree_name, const std::string & package_name = "");

  /**
   * @brief Find an installed behavior tree resource using a specific behavior tree XML file stem.
   *
   * This factory method acts equivalently to passing an identity string formatted like
   * `<package_name>::<tree_file_stem>::` to the constructor.
   * @param file_stem Stem of the desired behavior tree file (the stem of a file is the file name without the
   * extension).
   * @param package_name Optional package name provided to narrow down the search. If empty, search in all installed
   * packages.
   * @return Corresponding tree resource object.
   * @throws auto_apms_util::exceptions::ResourceError if the corresponding behavior tree cannot be found using the
   * given arguments.
   */
  static TreeResource selectByFileStem(const std::string & file_stem, const std::string & package_name = "");

  /**
   * @brief Determine if this behavior tree resource specifies a root tree.
   *
   * The name of the root tree is determined as follows:
   *
   * - If the `<tree_name>` token was present in the resource identity when this instance was created, this is
   * considered the root tree name.
   *
   * - Otherwise, the XML of the associated tree document is parsed to determine the root tree.
   *
   * @return `true` if the root tree of this resource can be determined, `false` otherwise.
   */
  bool hasRootTreeName() const;

  /**
   * @brief Get the name of the root tree of this behavior tree resource.
   *
   * The name of the root tree is determined as follows:
   *
   * - If the `<tree_name>` token was present in the resource identity when this instance was created, this is
   * considered the root tree name.
   *
   * - Otherwise, the XML of the associated tree document is parsed to determine the root tree.
   *
   * @return Name of this resource's root tree.
   * @throw auto_apms_util::exceptions::ResourceError if the name of the root tree cannot be determined.
   */
  std::string getRootTreeName() const;

  /**
   * @brief Get the node manifest associated with this resource.
   * @return Node manifest object.
   */
  NodeManifest getNodeManifest() const;

  /**
   * @brief Get the name of the package this resource was registered by.
   * @return Package name.
   */
  std::string getPackageName() const;

  /**
   * @brief Get the file stem of the XML file containing the tree document associated with this resource.
   * @return File stem of the associated XML file.
   */
  std::string getFileStem() const;

  /**
   * @brief Create a valid tree resource identity string representing this resource.
   * @param tree_name Name of one of the trees inside this resource's tree document the returned identity string
   should
   * refer to. If empty, do not refer to a specific behavior tree.
   * @return Tree resource identity string.
   */
  Identity createIdentity(const std::string & tree_name = "") const;

private:
  const BehaviorResourceIdentity identity_;
  std::string package_name_;
  std::string tree_file_path_;
  std::vector<std::string> node_manifest_file_paths_;
  std::string doc_root_tree_name_;
};

}  // namespace auto_apms_behavior_tree::core

/// @cond INTERNAL
namespace YAML
{
template <>
struct convert<auto_apms_behavior_tree::core::BehaviorResourceIdentity>
{
  using Identity = auto_apms_behavior_tree::core::BehaviorResourceIdentity;
  static Node encode(const Identity & rhs)
  {
    Node node;
    node = rhs.str();
    return node;
  }
  static bool decode(const Node & node, Identity & rhs)
  {
    if (!node.IsScalar()) return false;
    rhs = Identity(node.Scalar());
    return true;
  }
};
}  // namespace YAML
/// @endcond
