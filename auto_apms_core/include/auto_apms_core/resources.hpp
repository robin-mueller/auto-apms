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

#include "pluginlib/class_loader.hpp"
#include "auto_apms_core/util/string.hpp"
#include "auto_apms_core/exceptions.hpp"

namespace auto_apms_core
{

/**
 * @ingroup auto_apms_core
 * @brief Collect all package names that register a certain type of `ament_index` resources.
 *
 * \note Resources are not available until the respective ROS2 package is installed.
 *
 * @param resource_type Name of the resource type.
 * @return Package names.
 * @throws exceptions::ResourceNotFoundError if no resources of type @p resource_type were found.
 */
std::set<std::string> getAllPackagesWithResource(const std::string& resource_type);

/**
 * @ingroup auto_apms_core
 * @brief Create an instance of pluginlib::ClassLoader for loading installed plugins.
 *
 * This function requires packages to install plugins.xml manifest files and register them as an `ament_index` resource.
 *
 * @param base_package Name of the package containing the definition of the base class BaseT.
 * @param base_class_type Fully qualified name of the base class BaseT. This information is vital to be able to
 * determine relevant plugin data inside the manifest file. Only plugins specifying this class as their base will be
 * loaded.
 * @param resource_type Name of the `ament_index` resource type containing the path to the plugins.xml manifest file
 * relative to the package's install prefix.
 * @param search_packages Packages to consider when searching for plugin resources. Leave empty to search in all
 * packages.
 * @return Initialized class loader.
 * @throws auto_apms_core::exceptions::ResourceNotFoundError if failed to find a pluginlib plugin
 * manifest file in a package specified in @p search_packages or if a `ament_index` resource marker file is invalid.
 */
template <typename BaseT>
pluginlib::ClassLoader<BaseT> makePluginClassLoader(const std::string& base_package, const std::string& base_class_type,
                                                    const std::string& resource_type,
                                                    const std::set<std::string>& search_packages = {})
{
  {
    std::vector<std::string> xml_paths;
    for (const auto& name : search_packages.empty() ? getAllPackagesWithResource(resource_type) : search_packages)
    {
      std::string content;
      std::string base_path;
      if (ament_index_cpp::get_resource(resource_type, name, content, &base_path))
      {
        std::vector<std::string> paths = util::splitString(content, "\n", false);
        if (paths.size() != 1)
        {
          throw exceptions::ResourceNotFoundError("Invalid resource marker file installed by package: '" + name +
                                                  "' for resource type '" + resource_type +
                                                  "'. Must contain a single line with a path to the plugins.xml "
                                                  "manifest "
                                                  "file relative to the package's install prefix.");
        }
        xml_paths.push_back(base_path + '/' + paths[0]);
      }
      else
      {
        throw exceptions::ResourceNotFoundError("Cannot find any resources for type '" + resource_type +
                                                "' in install directory of package '" + name + "'.");
      }
    }
    return { base_package, base_class_type, "", xml_paths };
  }
}

}  // namespace auto_apms_core
