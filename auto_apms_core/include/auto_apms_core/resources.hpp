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
#include <vector>
#include <string>

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
 * @brief Collect the paths of plugin.xml manifest files used for initializing pluginlib::ClassLoader objects.
 *
 * This function requires packages to install plugins.xml manifest files and register them as an `ament_index` resource.
 *
 * @param resource_type Name of the `ament_index` resource type containing the path to the plugins.xml manifest file
 * relative to the package's install prefix.
 * @param search_packages Packages to consider when searching for plugin resources. Leave empty to search in all
 * packages.
 * @return Vector of file paths.
 * @throws auto_apms_core::exceptions::ResourceNotFoundError if failed to find a corresponding file in a package
 * specified in @p search_packages or if an `ament_index` resource marker file is invalid.
 */
std::vector<std::string> collectPluginXMLPaths(const std::string& resource_type,
                                               const std::set<std::string>& search_packages = {});

}  // namespace auto_apms_core
