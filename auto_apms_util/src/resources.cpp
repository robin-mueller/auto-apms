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

#include "auto_apms_util/resources.hpp"

#include "ament_index_cpp/get_resources.hpp"
#include "ament_index_cpp/get_resource.hpp"
#include "auto_apms_util/string.hpp"
#include "auto_apms_util/exceptions.hpp"

namespace auto_apms_util
{

std::set<std::string> getAllPackagesWithResource(const std::string& resource_type)
{
  std::set<std::string> all_packages;
  for (const auto& [package_name, _] : ament_index_cpp::get_resources(resource_type))
  {
    all_packages.insert(package_name);
  }
  if (all_packages.empty())
  {
    throw exceptions::ResourceNotFoundError("No resources of type '" + resource_type +
                                            "' were found in the installed packages.");
  }
  return all_packages;
}

std::vector<std::string> collectPluginXMLPaths(const std::string& resource_type,
                                               const std::set<std::string>& search_packages)
{
  std::vector<std::string> xml_paths;
  for (const auto& name : search_packages.empty() ? getAllPackagesWithResource(resource_type) : search_packages)
  {
    std::string content;
    std::string base_path;
    if (ament_index_cpp::get_resource(resource_type, name, content, &base_path))
    {
      std::vector<std::string> paths = splitString(content, "\n", false);
      if (paths.size() != 1)
      {
        throw exceptions::ResourceNotFoundError("Invalid resource marker file installed by package: '" + name +
                                                "' for resource type '" + resource_type +
                                                "'. Must contain a single line with a path to the plugins.xml "
                                                "manifest file relative to the package's install prefix.");
      }
      xml_paths.push_back(base_path + '/' + paths[0]);
    }
    else
    {
      throw exceptions::ResourceNotFoundError("Cannot find any resources for type '" + resource_type +
                                              "' in install directory of package '" + name + "'.");
    }
  }
  return xml_paths;
}

}  // namespace auto_apms_util
