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

#include "auto_apms_util/resource.hpp"

#include "ament_index_cpp/get_resource.hpp"
#include "ament_index_cpp/get_resources.hpp"
#include "auto_apms_util/exceptions.hpp"
#include "auto_apms_util/string.hpp"
#include "pluginlib/class_loader.hpp"

namespace auto_apms_util
{

std::set<std::string> getPackagesWithResource(
  const std::string & resource_type, const std::set<std::string> & exclude_packages)
{
  std::set<std::string> packages;
  for (const auto & [package, _] : ament_index_cpp::get_resources(resource_type)) {
    packages.insert(package);
  }
  if (packages.empty()) {
    throw exceptions::ResourceError(
      "Cannot find resources for type '" + resource_type + "' in any installed packages.");
  }
  if (const std::set<std::string> common = getCommonElements(packages, exclude_packages); !common.empty()) {
    for (const std::string & s : common) packages.erase(s);
    if (packages.empty()) {
      throw exceptions::ResourceError(
        "Resources for type '" + resource_type +
        "' are only available in excluded but not in any other installed packages (Relevant excluded packages: [ " +
        rcpputils::join(std::vector<std::string>(common.begin(), common.end()), ", ") + " ]).");
    }
  }
  return packages;
}

std::vector<std::string> collectPluginXMLPaths(
  const std::string & resource_type, const std::set<std::string> & exclude_packages)
{
  std::vector<std::string> xml_paths;
  for (const std::string & package : getPackagesWithResource(resource_type, exclude_packages)) {
    std::string content;
    std::string base_path;
    if (ament_index_cpp::get_resource(resource_type, package, content, &base_path)) {
      std::vector<std::string> paths = splitString(content, "\n", false);
      if (paths.size() != 1) {
        throw exceptions::ResourceError(
          "Invalid resource marker file installed by package: '" + package + "' for resource type '" + resource_type +
          "'. Must contain a single line with a path to the plugins.xml "
          "manifest file relative to the package's install prefix.");
      }
      xml_paths.push_back(base_path + '/' + paths[0]);
    } else {
      throw exceptions::ResourceError(
        "Cannot find any resources for type '" + resource_type + "' in package '" + package + "'.");
    }
  }
  return xml_paths;
}

}  // namespace auto_apms_util
