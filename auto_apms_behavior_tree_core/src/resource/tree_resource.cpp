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
        throw std::runtime_error(
          "Invalid behavior tree resource file (Package: '" + package_name + "'). Invalid line: " + line + ".");
      }
      TreeResource r;
      r.tree_file_stem = parts[0];
      r.tree_file_path = make_absolute_path(parts[1]);
      r.package_name = package_name;
      for (const std::string & path : auto_apms_util::splitString(parts[2], ";"))
        r.node_manifest_file_paths.push_back(make_absolute_path(path));
      std::vector<std::string> tree_ids_vec = auto_apms_util::splitString(parts[3], ";");
      r.tree_names = {tree_ids_vec.begin(), tree_ids_vec.end()};
      resources.push_back(r);
    }
  }
  return resources;
}

TreeResource TreeResource::selectByTreeName(const std::string & tree_name, const std::string & package_name)
{
  std::set<std::string> search_packages;
  if (!package_name.empty()) {
    search_packages.insert(package_name);
  } else {
    search_packages =
      auto_apms_util::getPackagesWithResourceType(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__TREE);
  }

  std::vector<TreeResource> matching_resources;
  for (const auto & package_name : search_packages) {
    for (const auto & r : collectFromPackage(package_name)) {
      if (r.tree_names.find(tree_name) != r.tree_names.end()) {
        matching_resources.push_back(r);
      }
    }
  }

  if (matching_resources.empty()) {
    throw auto_apms_util::exceptions::ResourceError{"No behavior tree with name '" + tree_name + "' was registered."};
  }
  if (matching_resources.size() > 1) {
    throw auto_apms_util::exceptions::ResourceError{
      "The behavior tree name '" + tree_name +
      "' exists multiple times. Use the 'package_name' argument "
      "to narrow down the search."};
  }

  return matching_resources[0];
}

TreeResource TreeResource::selectByFileName(const std::string & file_name, const std::string & package_name)
{
  const std::string file_stem = std::filesystem::path{file_name}.stem().string();
  std::set<std::string> search_packages;
  if (!package_name.empty()) {
    search_packages.insert(package_name);
  } else {
    search_packages =
      auto_apms_util::getPackagesWithResourceType(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__TREE);
  }

  std::vector<TreeResource> matching_resources;
  for (const auto & package_name : search_packages) {
    for (const auto & r : collectFromPackage(package_name)) {
      if (r.tree_file_stem == file_stem) {
        matching_resources.push_back(r);
      }
    }
  }

  if (matching_resources.empty()) {
    throw auto_apms_util::exceptions::ResourceError{
      "No behavior tree file with name '" + file_stem + ".xml' was registered."};
  }
  if (matching_resources.size() > 1) {
    throw auto_apms_util::exceptions::ResourceError{
      "Multiple behavior tree files with name '" + file_stem +
      ".xml' are registered. Use the 'package_name' argument to "
      "narrow down the search."};
  }

  return matching_resources[0];
}

TreeResource TreeResource::fromString(const std::string & identity)
{
  const auto tokens = auto_apms_util::splitString(identity, "::", false);
  if (tokens.size() != 3) {
    throw auto_apms_behavior_tree::exceptions::ResourceIdentityFormatError(
      "Identity string '" + identity +
      "' has wrong format. Number of string tokens separated by '::' must "
      "be 3.");
  }
  const std::string & tree_file_stem = tokens[0];
  const std::string & tree_name = tokens[1];
  const std::string & package_name = tokens[2];
  if (tree_name.empty()) return selectByFileName(tree_file_stem, package_name);
  if (tree_file_stem.empty()) return selectByTreeName(tree_name, package_name);

  // Full signature: Verify that <tree_name> can be found in file with stem <tree_file_stem>
  TreeResource resource = selectByFileName(tree_file_stem, package_name);
  if (resource.tree_names.find(tree_name) == resource.tree_names.end()) {
    throw auto_apms_util::exceptions::ResourceError(
      "Found behavior tree file '" + tree_file_stem + ".xml' in package '" + resource.package_name +
      "' but no tree with name '" + tree_name + "' exists in that file.");
  }
  return resource;
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