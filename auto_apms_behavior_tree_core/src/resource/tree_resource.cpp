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

#include "auto_apms_behavior_tree_core/resource/tree_resource.hpp"

#include <tinyxml2.h>

#include <filesystem>

#include "ament_index_cpp/get_resource.hpp"
#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_util/exceptions.hpp"
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

TreeResource::TreeResource(const TreeResourceIdentity & identity)
{
  std::set<std::string> search_packages;
  if (!identity.package_name.empty()) {
    search_packages.insert(identity.package_name);
  } else {
    search_packages =
      auto_apms_util::getPackagesWithResourceType(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__TREE);
  }

  std::vector<TreeResource> matching_resources;
  for (const auto & package_name : search_packages) {
    for (const auto & r : collectFromPackage(package_name)) {
      if (identity.tree_name.empty()) {
        if (r.tree_file_stem != identity.file_stem) continue;
      } else if (identity.file_stem.empty()) {
        if (r.tree_names.find(identity.tree_name) == r.tree_names.end()) continue;
      } else {
        if (r.tree_file_stem != identity.file_stem && r.tree_names.find(identity.tree_name) == r.tree_names.end())
          continue;
      }
      matching_resources.push_back(r);
    }
  }

  if (matching_resources.empty()) {
    throw auto_apms_util::exceptions::ResourceError(
      "No behavior tree file with identity '" + identity.str() + "' was registered.");
  }
  if (matching_resources.size() > 1) {
    throw auto_apms_util::exceptions::ResourceError(
      "Resource identity '" + identity.str() + "' is ambiguous. You must be more precise.");
  }

  const TreeResource & resource = matching_resources[0];
  package_name = resource.package_name;
  tree_file_stem = resource.tree_file_stem;
  tree_names = resource.tree_names;
  tree_file_path = resource.tree_file_path;
  node_manifest_file_paths = resource.node_manifest_file_paths;
}

std::vector<TreeResource> TreeResource::collectFromPackage(const std::string & package_name)
{
  std::string content;
  std::string base_path;
  std::vector<TreeResource> resources;
  if (ament_index_cpp::get_resource(
        _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__TREE, package_name, content, &base_path)) {
    std::vector<std::string> lines = auto_apms_util::splitString(content, "\n");
    auto make_absolute_path = [base_path](const std::string & s) { return base_path + "/" + s; };
    for (const auto & line : lines) {
      std::vector<std::string> parts = auto_apms_util::splitString(line, "|", false);
      if (parts.size() != 4) {
        throw auto_apms_util::exceptions::ResourceError(
          "Invalid behavior tree resource file (Package: '" + package_name + "'). Invalid line: " + line + ".");
      }
      TreeResource r;
      r.package_name = package_name;
      r.tree_file_stem = parts[0];
      std::vector<std::string> tree_ids_vec = auto_apms_util::splitString(parts[1], ";");
      r.tree_names = {tree_ids_vec.begin(), tree_ids_vec.end()};
      r.tree_file_path = make_absolute_path(parts[2]);
      for (const std::string & path : auto_apms_util::splitString(parts[3], ";")) {
        r.node_manifest_file_paths.push_back(
          std::filesystem::path(path).is_absolute() ? path : make_absolute_path(path));
      }
      resources.push_back(r);
    }
  }
  return resources;
}

TreeResource TreeResource::selectByTreeName(const std::string & tree_name, const std::string & package_name)
{
  return TreeResource(package_name + "::::" + tree_name);
}

TreeResource TreeResource::selectByFileName(const std::string & file_name, const std::string & package_name)
{
  return TreeResource(package_name + "::" + file_name + "::");
}

std::string TreeResource::getRootTreeName(const std::string & root_tree_attribute_name) const
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(tree_file_path.c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
    throw auto_apms_behavior_tree::exceptions::TreeXMLFormatError(
      "Invalid tree xml in resource file " + tree_file_path + ": " + doc.ErrorStr());
  }
  if (const auto root_tree_name = doc.RootElement()->Attribute(root_tree_attribute_name.c_str())) return root_tree_name;
  return "";
}

std::string TreeResource::writeTreeToString() const
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(tree_file_path.c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
    throw auto_apms_behavior_tree::exceptions::TreeXMLFormatError(
      "Invalid tree xml in resource file " + tree_file_path + ": " + doc.ErrorStr());
  }
  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  return printer.CStr();
}

}  // namespace auto_apms_behavior_tree::core