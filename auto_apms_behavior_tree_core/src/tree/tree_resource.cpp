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

#include "auto_apms_behavior_tree_core/tree/tree_resource.hpp"

#include <tinyxml2.h>

#include <filesystem>

#include "ament_index_cpp/get_resource.hpp"
#include "auto_apms_behavior_tree_core/definitions.hpp"
#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "auto_apms_util/resource.hpp"
#include "auto_apms_util/string.hpp"

namespace auto_apms_behavior_tree::core
{

TreeResourceIdentity::TreeResourceIdentity(const std::string & identity)
{
  std::vector<std::string> tokens = auto_apms_util::splitString(identity, "::", false);
  if (tokens.size() == 1) {
    tokens.insert(tokens.begin(), "");
    tokens.push_back("");
  }
  if (tokens.size() != 3) {
    throw auto_apms_util::exceptions::ResourceIdentityFormatError(
      "Identity string '" + identity + "' has wrong format. Number of string tokens separated by '::' must be 3.");
  }
  package_name = tokens[0];
  file_stem = tokens[1];
  tree_name = tokens[2];
  if (file_stem.empty() && tree_name.empty()) {
    throw auto_apms_util::exceptions::ResourceIdentityFormatError(
      "Behavior tree resource identity string '" + identity +
      "' has wrong format. It's not allowed to omit both <tree_file_stem> and <tree_name>.");
  }
}

std::string TreeResourceIdentity::str() const { return package_name + "::" + file_stem + "::" + tree_name; }

TreeResource::TreeResource(const TreeResourceIdentity & identity) : identity_(identity)
{
  std::set<std::string> search_packages;
  if (!identity.package_name.empty()) {
    search_packages.insert(identity.package_name);
  } else {
    search_packages =
      auto_apms_util::getPackagesWithResourceType(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__TREE);
  }

  size_t matching_count = 0;
  std::string matching_package_name;
  std::string matching_tree_file_stem;
  std::set<std::string> matching_tree_names;
  std::string matching_tree_file_path;
  std::vector<std::string> matching_node_manifest_file_paths;
  for (const auto & p : search_packages) {
    std::string content;
    std::string base_path;
    if (ament_index_cpp::get_resource(
          _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__TREE, p, content, &base_path)) {
      for (const auto & line : auto_apms_util::splitString(content, "\n")) {
        const std::vector<std::string> parts = auto_apms_util::splitString(line, "|", false);
        if (parts.size() != 4) {
          throw auto_apms_util::exceptions::ResourceError(
            "Invalid behavior tree resource file (Package: '" + p + "'). Invalid line: " + line + ".");
        }
        const std::string & found_tree_file_stem = parts[0];
        const std::vector<std::string> found_tree_names = auto_apms_util::splitString(parts[1], ";");

        // Determine if resource is matching
        if (identity.tree_name.empty()) {
          if (found_tree_file_stem != identity.file_stem) {
            continue;
          }
        } else if (identity.file_stem.empty()) {
          if (!auto_apms_util::contains(found_tree_names, identity.tree_name)) {
            continue;
          }
        } else if (
          found_tree_file_stem != identity.file_stem ||
          !auto_apms_util::contains(found_tree_names, identity.tree_name)) {
          continue;
        }

        matching_count++;
        matching_package_name = p;
        matching_tree_file_stem = found_tree_file_stem;
        matching_tree_names = {found_tree_names.begin(), found_tree_names.end()};
        matching_tree_file_path = base_path + "/" + parts[2];
        for (const std::string & path : auto_apms_util::splitString(parts[3], ";")) {
          matching_node_manifest_file_paths.push_back(
            std::filesystem::path(path).is_absolute() ? path : (base_path + "/" + path));
        }
      }
    }
  }

  if (matching_count == 0) {
    throw auto_apms_util::exceptions::ResourceError(
      "No behavior tree file with identity '" + identity.str() + "' was registered.");
  }
  if (matching_count > 1) {
    throw auto_apms_util::exceptions::ResourceError(
      "Resource identity '" + identity.str() + "' is ambiguous. You must be more precise.");
  }

  // Set the members since we've found the matching resource
  package_name_ = matching_package_name;
  tree_file_path_ = matching_tree_file_path;
  node_manifest_file_paths_ = matching_node_manifest_file_paths;

  // Verify that the file is ok
  TreeDocument doc;
  try {
    doc.mergeFile(tree_file_path_);
  } catch (const std::exception & e) {
    throw auto_apms_util::exceptions::ResourceError(
      "Failed to create TreeResource with identity '" + identity.str() + "' because tree file " + tree_file_path_ +
      " cannot be parsed: " + e.what());
  }

  // Verify that the tree <tree_name> specified by the identity string is actually present
  if (!identity.tree_name.empty()) {
    if (auto_apms_util::contains(doc.getAllTreeNames(), identity.tree_name)) {
      throw auto_apms_util::exceptions::ResourceError(
        "Cannot create TreeResource with identity '" + identity.str() + "' because '" + identity.tree_name +
        "' does not exist in tree file " + tree_file_path_ + ".");
    }
  }
}

TreeResource::TreeResource(const std::string & identity) : TreeResource(TreeResourceIdentity(identity)) {}

TreeResource::TreeResource(const char * identity) : TreeResource(std::string(identity)) {}

TreeResource TreeResource::selectByTreeName(const std::string & tree_name, const std::string & package_name)
{
  return TreeResource(package_name + "::::" + tree_name);
}

TreeResource TreeResource::selectByFileName(const std::string & file_name, const std::string & package_name)
{
  return TreeResource(package_name + "::" + file_name + "::");
}

std::string TreeResource::getRootTreeName() const
{
  if (!identity_.tree_name.empty()) return identity_.tree_name;

  // If <tree_name> wasn't provided, look for root tree attribute in XML file
  TreeDocument doc;
  if (doc.mergeResource(*this, true).hasRootTreeName()) return doc.getRootTreeName();

  // Root tree cannot be determined
  throw auto_apms_util::exceptions::ResourceError(
    "Cannot get root tree name of tree resource '" + identity_.str() + "'. Since there is no XML attribute named '" +
    TreeDocument::ROOT_TREE_ATTRIBUTE_NAME +
    "' and the resource identity doesn't specify <tree_name>, the root tree is unkown.");
}

NodeManifest TreeResource::getNodeManifest() const { return NodeManifest::fromFiles(node_manifest_file_paths_); }

std::string TreeResource::getPackageName() const { return package_name_; }

std::string TreeResource::getFileStem() const { return std::filesystem::path(tree_file_path_).stem(); }

std::string TreeResource::str() const { return identity_.str(); }

}  // namespace auto_apms_behavior_tree::core