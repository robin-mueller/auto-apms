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

#include "auto_apms_behavior_tree_core/behavior.hpp"
#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include "auto_apms_util/yaml.hpp"

namespace auto_apms_behavior_tree::core
{

/**
 * @brief Struct that encapsulates the identity string for a declared behavior tree.
 *
 * Its only purpose is to create the corresponding instance of TreeResource.
 */
struct TreeResourceIdentity : public BehaviorResourceIdentity
{
  /**
   * @brief Constructor of a tree resource identity object.
   *
   * @p identity must be formatted like `<package_name>::<tree_file_stem>::<tree_name>`.
   * @param identity Identity string for a specific behavior tree resource.
   * @throws auto_apms_util::exceptions::ResourceIdentityFormatError if the identity string has wrong format.
   */
  TreeResourceIdentity(const std::string & identity);

  /**
   * @brief Constructor of a tree resource identity object.
   *
   * @p identity must be formatted like `<package_name>::<tree_file_stem>::<tree_name>`.
   * @param identity C-style identity string for a specific behavior tree resource.
   * @throws auto_apms_util::exceptions::ResourceIdentityFormatError if the identity string has wrong format.
   */
  TreeResourceIdentity(const char * identity);

  /**
   * @brief Constructor of an empty behavior tree resource identity object.
   *
   * The user must manually populate the member fields according to the behavior tree this object should identify.
   */
  TreeResourceIdentity() = default;

  /// Name of the file (without extension) that contains the resource's tree document.
  std::string file_stem;
  /// Name of a specific tree inside the resource's tree document.
  std::string tree_name;
};

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Class containing behavior tree resource data
 *
 * Behavior tree resources are registered by calling the CMake macro `auto_apms_behavior_tree_declare_trees` in the
 * CMakeLists.txt of a package. They can be discovered once the corresponding package has been installed to the ROS 2
 * workspace.
 *
 * The user may refer to a specific resource using an identity string that may contain the tokens `<package_name>`,
 * `<tree_file_stem>` and `<tree_name>` separated by `::` in that order. Depending on the number of registered
 * resources, it might be convenient to use shorter, less precise signatures. Additionally, if the delimiter `::` is not
 * present, the string is assumed to be the stem of a behavior tree file `<tree_file_stem>`. All possible identity
 * strings are listed below:
 *
 * - `<package_name>::<tree_file_stem>::<tree_name>`
 *
 *   Fully qualified identity string of a specific behavior tree.
 *
 * - `::<tree_file_stem>::<tree_name>`
 *
 *   Try to find the resource by searching for a tree with name `<tree_name>` in a file with stem `<tree_file_stem>`
 *   considering all packages.
 *
 * - `<package_name>::::<tree_name>`
 *
 *   Try to find a tree with name `<tree_name>` within the resources registered by `<package_name>`.
 *
 * - `::::<tree_name>`
 *
 *   Try to find the resource by searching for a tree with name `<tree_name>` considering all packages.
 *
 * - `<package_name>::<tree_file_stem>::`
 *
 *   Try to find a file with stem `<tree_file_stem>` within the resources registered by `<package_name>`.
 *
 * - `::<tree_file_stem>::` or `<tree_file_stem>`
 *
 *   Try to find the resource by searching for a file with stem `<tree_file_stem>` considering all packages. Here you
 *   may conveniently omit the `::` delimiter, since this is the most common way for searching for a resource.
 *
 * @note The delimiter `::` must be kept when tokens are omitted, except when searching for resources using only the
 * file stem.
 *
 * ## Usage
 *
 * Given the user has specified a behavior tree named `MyBehaviorTree` inside the XML file `config/my_tree_file.xml`,
 * the CMake macro `auto_apms_behavior_tree_declare_trees` must be called in the CMakeLists.txt of the parent package
 * (for example `my_package`) like this:
 *
 * ```cmake
 * auto_apms_behavior_tree_declare_trees(
 *     "config/my_tree_file.xml"
 * )
 * ```
 *
 * The macro automatically parses the given files and detects the names of the trees inside. In the C++ source code, one
 * may use this resource, after the parent package `my_package` has been installed, like this:
 *
 * ```cpp
 * #include "auto_apms_behavior_tree_core/tree/tree_resource.hpp"
 *
 * using namespace auto_apms_behavior_tree;
 *
 * // For example, use the fully qualified tree resource identity signature
 * const std::string identity_string = "my_package::my_behavior_tree::MyTreeName";
 *
 * // You may use the proxy class for a tree resource identity
 * core::TreeResourceIdentity identity(identity_string);
 * core::TreeResource resource(identity);
 *
 * // Or instantiate the resource object directly from the corresponding identity string
 * core::TreeResource resource(identity_string);
 *
 * // The resource object may for example be used with TreeDocument
 * core::TreeDocument doc;
 * doc.mergeResource(resource);  // Add "MyTreeName" to the document
 *
 * // This also works, so creating a resource object is not strictly necessary
 * doc.mergeResource(identity_string)
 *
 * // The simplest approach is this
 * doc.mergeResource("my_package::my_behavior_tree::MyTreeName");
 * ```
 *
 */
class TreeResource
{
  friend class TreeDocument;
  friend class TreeBuilder;

public:
  using Identity = TreeResourceIdentity;

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
   * @throws auto_apms_util::exceptions::ResourceError if the resource cannot be found using the given identity string.
   */
  TreeResource(const std::string & identity);

  /**
   * @brief Assemble a behavior tree resource identified by a string.
   *
   * @p identity must be formatted like `<package_name>::<tree_file_stem>::<tree_name>`.
   * @param identity C-style identity string for a specific behavior tree resource.
   * @throws auto_apms_util::exceptions::ResourceIdentityFormatError if the identity string has wrong format.
   * @throws auto_apms_util::exceptions::ResourceError if the resource cannot be found using the given identity string.
   */
  TreeResource(const char * identity);

  /**
   * @brief Find an installed behavior tree resource using a specific behavior tree name.
   *
   * This factory method acts equivalently to passing an identity string formatted like `<package_name>::::<tree_name>`
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
   * @param tree_name Name of one of the trees inside this resource's tree document the returned identity string should
   * refer to. If empty, do not refer to a specific behavior tree.
   * @return Tree resource identity string.
   */
  Identity createIdentity(const std::string & tree_name = "") const;

private:
  const TreeResourceIdentity identity_;
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
struct convert<auto_apms_behavior_tree::core::TreeResourceIdentity>
{
  using Identity = auto_apms_behavior_tree::core::TreeResourceIdentity;
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
