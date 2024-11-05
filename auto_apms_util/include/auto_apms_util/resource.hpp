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

#include <string>
#include <vector>
#include <set>
#include <map>

#include "pluginlib/class_loader.hpp"
#include "auto_apms_util/container.hpp"
#include "auto_apms_util/string.hpp"
#include "auto_apms_util/exceptions.hpp"

namespace auto_apms_util
{

/// @ingroup auto_apms_util
/// @{

/**
 * @brief Collect all package names that register a certain type of `ament_index` resources.
 *
 * \note Resources are not available until the respective ROS2 package is installed.
 *
 * @param resource_type Name of the resource type.
 * @return Package names.
 * @throws exceptions::ResourceError if no resources of type @p resource_type were found.
 */
std::set<std::string> getAllPackagesWithResource(const std::string& resource_type);

/**
 * @brief Collect the paths of plugin.xml manifest files used for initializing pluginlib::ClassLoader objects.
 *
 * This function requires packages to install plugins.xml manifest files and register them as an `ament_index` resource.
 * The resource marker file must contain the path to the plugins.xml manifest file relative to the package's install
 * prefix.
 *
 * @param resource_type Name of the `ament_index` resource type associated with the plugins.xml file paths to collect.
 * @param search_packages Packages to consider when searching for plugins.xml file paths. Leave empty to search in all
 * packages.
 * @return Vector of file paths.
 * @throws auto_apms_util::exceptions::ResourceError if failed to find a plugin manifest file in a package specified
 * in @p search_packages.
 * @throw auto_apms_util::exceptions::ResourceError if an `ament_index` resource marker file is invalid.
 */
std::vector<std::string> collectPluginXMLPaths(const std::string& resource_type,
                                               const std::set<std::string>& search_packages = {});

/// @}

/**
 * @ingroup auto_apms_util
 * @brief Class for loading plugin resources registered according to the conventions defined by the pluginlib package.
 * @tparam BaseT Base class of the plugin.
 */
template <typename BaseT>
class ResourceClassLoader : public pluginlib::ClassLoader<BaseT>
{
public:
  /**
   * @brief Standard ResourceClassLoader constructor.
   *
   * Alternatively, if you want to verify that only unique class names are loaded, you can refer to
   * ResourceClassLoader::createWithAmbiguityCheck.
   *
   * @param base_package  Name of the package containing the @p base_class. Will throw an error if it is not
   * installed.
   * @param base_class Fully qualified name of the base class that the class loader will use for determining which
   * classes listed in a plugins.xml file are associated with this instance.
   * @param plugin_xml_paths List of plugins.xml files to parse for determining the available classes.
   * @throws ament_index_cpp::PackageNotFoundError if @p base_package is not installed.
   */
  ResourceClassLoader(const std::string& base_package, const std::string& base_class,
                      const std::vector<std::string>& plugin_xml_paths);

  /**
   * @brief Parse all associated plugin manifest files registered with the ament resource index and instantiate a
   * ResourceClassLoader.
   *
   * This factory method additionally performs an ambiguity check. This means that it verifies that only unique class
   * names are being registered by the searched packages. Using the standard constructor, previously registered classes
   * are being overriden if the same class name is being parsed again. Therefore, this method offers the
   * preferred way of instantiating a ResourceClassLoader.
   *
   * @param base_package Name of the package containing the @p base_class. Will throw an error if it is not
   * installed.
   * @param base_class Fully qualified name of the base class that the class loader will use for determining which
   * classes listed in a plugins.xml file are associated with this instance.
   * @param resource_type Name of the `ament_index` resource type associated with the class loader.
   * @param search_packages Packages to consider when searching for plugins. Leave empty to search in all packages.
   * @param reserved_names Map of reserved class names and the package name that makes the reservation. If any of these
   * class names are found, an error is being raised (Used internally during build time when installed resources aren't
   * availabel yet).
   * @throws auto_apms_util::exceptions::ResourceError if multiple packages register a resource using the same class
   * name (or a name in @p reserved_names is found).
   * @sa
   * - ResourceClassLoader::ResourceClassLoader()
   * - collectPluginXMLPaths()
   */
  static ResourceClassLoader createWithAmbiguityCheck(const std::string& base_package, const std::string& base_class,
                                                      const std::string& resource_type,
                                                      const std::set<std::string>& search_packages = {},
                                                      const std::map<std::string, std::string>& reserved_names = {});
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

template <typename BaseT>
inline ResourceClassLoader<BaseT>::ResourceClassLoader(const std::string& base_package, const std::string& base_class,
                                                       const std::vector<std::string>& plugin_xml_paths)
  : pluginlib::ClassLoader<BaseT>(base_package, base_class, "", plugin_xml_paths)
{
}

template <typename BaseT>
inline ResourceClassLoader<BaseT> ResourceClassLoader<BaseT>::createWithAmbiguityCheck(
    const std::string& base_package, const std::string& base_class, const std::string& resource_type,
    const std::set<std::string>& search_packages, const std::map<std::string, std::string>& reserved_names)
{
  const auto packages_with_resource =
      search_packages.empty() ? getAllPackagesWithResource(resource_type) : search_packages;
  std::map<std::string, std::vector<std::string>> packages_for_class_name;
  for (const auto& package : packages_with_resource)
  {
    const std::vector<std::string> loader_names =
        pluginlib::ClassLoader<BaseT>(base_package, base_class, "", collectPluginXMLPaths(resource_type, { package }))
            .getDeclaredClasses();
    for (const auto& class_name : loader_names)
    {
      packages_for_class_name[class_name].push_back(package);
    }
  }

  // Include reserved names in map
  for (const auto& [class_name, package] : reserved_names)
  {
    packages_for_class_name[class_name].push_back(package);
  }

  // Determine if there are duplicate class names
  std::vector<std::string> error_details;
  for (const auto& [class_name, packages] : packages_for_class_name)
  {
    if (packages.size() > 1)
    {
      error_details.push_back("- Class '" + class_name + "' found in packages ['" + rcpputils::join(packages, "', '") +
                              "'].");
    }
  }
  if (!error_details.empty())
  {
    throw exceptions::ResourceError("Ambiguous class names found! ResourceClassLoader (Base: '" + base_class +
                                    "') created with createWithAmbiguityCheck() won't register resources from packages "
                                    "that use already existing lookup names. Found the following duplicates:\n" +
                                    rcpputils::join(error_details, "\n"));
  }
  return { base_package, base_class, collectPluginXMLPaths(resource_type, packages_with_resource) };
}

}  // namespace auto_apms_util
