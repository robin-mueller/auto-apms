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

#include "auto_apms_behavior_tree/resource/tree_resource.hpp"

#include <tinyxml2.h>

#include <filesystem>

#include "ament_index_cpp/get_resource.hpp"
#include "auto_apms_core/exceptions.hpp"
#include "auto_apms_core/resources.hpp"
#include "auto_apms_core/util/split.hpp"

namespace auto_apms_behavior_tree
{

std::vector<TreeResource> TreeResource::CollectFromPackage(const std::string& package_name)
{
  std::string content;
  std::string base_path;
  std::vector<TreeResource> resources;
  if (ament_index_cpp::get_resource(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__TREE, package_name, content,
                                    &base_path))
  {
    std::vector<std::string> lines = auto_apms_core::util::SplitString(content, "\n", false);
    auto make_absolute_path = [base_path](const std::string& s) { return base_path + "/" + s; };
    for (const auto& line : lines)
    {
      std::vector<std::string> parts = auto_apms_core::util::SplitString(line, "|");
      if (parts.size() != 4)
      {
        throw std::runtime_error("Invalid behavior tree resource file (Package: '" + package_name + "').");
      }
      TreeResource r;
      r.tree_file_stem = parts[0];
      r.tree_file_path = make_absolute_path(parts[1]);
      r.package_name = package_name;
      r.node_manifest_file_path = make_absolute_path(parts[2]);
      std::vector<std::string> tree_ids_vec = auto_apms_core::util::SplitString(parts[3], ";");
      r.tree_names = { tree_ids_vec.begin(), tree_ids_vec.end() };
      resources.push_back(r);
    }
  }
  return resources;
}

TreeResource TreeResource::SelectByTreeName(const std::string& tree_name, const std::string& package_name)
{
  std::set<std::string> search_packages;
  if (!package_name.empty())
  {
    search_packages.insert(package_name);
  }
  else
  {
    search_packages = auto_apms_core::GetAllPackagesWithResource(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__TREE);
  }

  std::vector<TreeResource> matching_resources;
  for (const auto& package_name : search_packages)
  {
    for (const auto& r : CollectFromPackage(package_name))
    {
      if (r.tree_names.find(tree_name) != r.tree_names.end())
      {
        matching_resources.push_back(r);
      }
    }
  }

  if (matching_resources.empty())
  {
    throw auto_apms_core::exceptions::ResourceNotFoundError{ "No behavior tree with name '" + tree_name +
                                                             "' was registered." };
  }
  if (matching_resources.size() > 1)
  {
    throw auto_apms_core::exceptions::ResourceNotFoundError{ "The behavior tree name '" + tree_name +
                                                             "' exists multiple times. Use the 'package_name' argument "
                                                             "to narrow down the search." };
  }

  return matching_resources[0];
}

TreeResource TreeResource::SelectByFileName(const std::string& file_name, const std::string& package_name)
{
  const std::string file_stem = std::filesystem::path{ file_name }.stem().string();
  std::set<std::string> search_packages;
  if (!package_name.empty())
  {
    search_packages.insert(package_name);
  }
  else
  {
    search_packages = auto_apms_core::GetAllPackagesWithResource(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__TREE);
  }

  std::vector<TreeResource> matching_resources;
  for (const auto& package_name : search_packages)
  {
    for (const auto& r : CollectFromPackage(package_name))
    {
      if (r.tree_file_stem == file_stem)
      {
        matching_resources.push_back(r);
      }
    }
  }

  if (matching_resources.empty())
  {
    throw auto_apms_core::exceptions::ResourceNotFoundError{ "No behavior tree file with name '" + file_stem +
                                                             ".xml' was registered." };
  }
  if (matching_resources.size() > 1)
  {
    throw auto_apms_core::exceptions::ResourceNotFoundError{ "Multiple behavior tree files with name '" + file_stem +
                                                             ".xml' are registered. Use the 'package_name' argument to "
                                                             "narrow down the search." };
  }

  return matching_resources[0];
}

TreeResource TreeResource::FromString(const std::string& identity)
{
  const auto tokens = auto_apms_core::util::SplitString(identity, "::");
  if (tokens.size() != 3)
  {
    throw exceptions::ResourceIdentityFormatError("Identity string '" + identity +
                                                  "' has wrong format. Number of string tokens separated by '::' must "
                                                  "be 3.");
  }
  const std::string& tree_file_stem = tokens[0];
  const std::string& tree_name = tokens[1];
  const std::string& package_name = tokens[2];
  if (tree_name.empty())
    return SelectByFileName(tree_file_stem, package_name);
  if (tree_file_stem.empty())
    return SelectByTreeName(tree_name, package_name);

  // Full signature: Verify that <tree_name> can be found in file with stem <tree_file_stem>
  TreeResource resource = SelectByFileName(tree_file_stem, package_name);
  if (resource.tree_names.find(tree_name) == resource.tree_names.end())
  {
    throw auto_apms_core::exceptions::ResourceNotFoundError(
        "Found behavior tree file '" + tree_file_stem + ".xml' in package '" + resource.package_name +
        "' but no tree with name '" + tree_name + "' exists in that file.");
  }
  return resource;
}

std::string TreeResource::WriteTreeToString() const
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(tree_file_path.c_str()) != tinyxml2::XMLError::XML_SUCCESS)
  {
    throw exceptions::TreeXMLFormatError("Invalid tree xml in resource file " + tree_file_path + ": " + doc.ErrorStr());
  }
  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  return printer.CStr();
}

}  // namespace auto_apms_behavior_tree