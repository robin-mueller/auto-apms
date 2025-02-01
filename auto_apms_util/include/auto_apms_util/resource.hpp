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

#include <map>
#include <set>
#include <string>
#include <type_traits>
#include <vector>

#include "auto_apms_util/container.hpp"
#include "auto_apms_util/exceptions.hpp"
#include "auto_apms_util/string.hpp"
#include "pluginlib/class_loader.hpp"

namespace auto_apms_util
{

/// @ingroup auto_apms_util
/// @{

/**
 * @brief Get a list of all package names that register a certain type of `ament_index` resources.
 *
 * \note This function determines what packages register resources by parsing the install directory, so any resources
 * that are not installed at the time this function is called won't be considered.
 *
 * @param resource_type Name of the resource type.
 * @param exclude_packages Packages to exclude when searching for resources.
 * @return List of all packages that register resources of type @p resource_type excluding @p exclude_packages.
 * @throws auto_apms_util::exceptions::ResourceError if no resources of type @p resource_type were found in any of the
 * intalled packages.
 */
std::set<std::string> getPackagesWithResourceType(
  const std::string & resource_type, const std::set<std::string> & exclude_packages = {});

/**
 * @brief Get a list of all package names that register AutoAPMS plugin resources.
 *
 * \note This function determines what packages register resources by parsing the install directory, so any resources
 * that are not installed at the time this function is called won't be considered.
 *
 * @param exclude_packages Packages to exclude when searching for resources.
 * @return List of all packages that register AutoAPMS plugins excluding @p exclude_packages.
 * @throws auto_apms_util::exceptions::ResourceError if no AutoAPMS plugin resources were found in any of the installed
 * packages.
 */
std::set<std::string> getPackagesWithPluginResources(const std::set<std::string> & exclude_packages = {});

/**
 * @brief Get the path of a plugin.xml manifest file used for initializing pluginlib::ClassLoader objects.
 *
 * This function requires packages to install plugins.xml manifest files and register them as an `ament_index` resource
 * using the CMake macro auto_apms_util_register_plugins().
 *
 * @param package Name of the package registering AutoAPMS plugin resources.
 * @return Absolute path to the `pluginlib`-style plugin manifest xml file.
 * @throws auto_apms_util::exceptions::ResourceError if failed to find a plugin manifest file.
 * @throw auto_apms_util::exceptions::ResourceError if an `ament_index` resource marker file is invalid.
 */
std::string getPluginXMLPath(const std::string & package);

/**
 * @brief Collect the paths of plugin.xml manifest files used for initializing pluginlib::ClassLoader objects.
 *
 * This function requires packages to install plugins.xml manifest files and register them as an `ament_index` resource
 * using the CMake macro auto_apms_util_register_plugins().
 *
 * @param exclude_packages Packages to exclude when searching for plugins.xml file paths.
 * @return Absolute paths to the `pluginlib`-style plugin manifest xml files.
 * @throws auto_apms_util::exceptions::ResourceError if failed to find a plugin manifest file.
 * @throw auto_apms_util::exceptions::ResourceError if an `ament_index` resource marker file is invalid.
 */
std::vector<std::string> collectPluginXMLPaths(const std::set<std::string> & exclude_packages = {});

/// @}

// clang-format off
/**
 * @ingroup auto_apms_util
 * @brief Class for loading plugin resources registered according to the conventions defined by the
 * [pluginlib](https://wiki.ros.org/pluginlib) package.
 * @tparam BaseT Base class of the plugin.
 *
 * For a class to be discoverable, you must call the C++ macro `PLUGINLIB_EXPORT_CLASS` and provide the arguments
 * according to your definitions. AutoAPMS provides convenient wrappers that encapsulate this macro.
 *
 * You must also make sure, that you create a plugins.xml file, add the corresponding entries and export it in the
 * CMakeLists.txt of your ROS 2 package. Again, AutoAPMS provides convenient wrapper macros for specific use cases.
 *
 * If you did all the above and want to instantiate a plugin class, you must call e.g. createSharedInstance and provide
 * a lookup name for the plugin formatted like this:
 *
 * - `<namespace>::<class_name>`
 *
 * | Token Name | Description |
 * | :---: | :--- |
 * | `<namespace>` | Full C++ namespace that the class `<class_name>` can be found in. It can be flat `foo` or arbitrarily nested `foo::bar` (individual levels must be separated by `::`). |
 * | `<class_name>` | Name you specified when declaring the plugin class using the `class` keyword in C++. If the class takes template arguments, you must provide it using the `MyClass<T>` syntax as you would in the source code. |
 */
// clang-format on
template <typename BaseT>
class PluginClassLoader : public pluginlib::ClassLoader<BaseT>
{
public:
  /**
   * @brief Standard PluginClassLoader constructor.
   *
   * Alternatively, if you want to verify that only unique class names are loaded, have a look at
   * PluginClassLoader::makeUnambiguousPluginClassLoader.
   *
   * @param base_package Name of the package containing the @p base_class. Will throw an error if it is not
   * installed.
   * @param base_class Fully qualified name of the base class that the class loader will use for determining which
   * classes listed in a plugins.xml file are associated with this instance.
   * @param exclude_packages Packages to exclude when searching for plugins.
   * @throws ament_index_cpp::PackageNotFoundError if @p base_package is not installed.
   * @throws auto_apms_util::exceptions::ResourceError if no plugins of type @p base_class were found.
   */
  PluginClassLoader(
    const std::string & base_package, const std::string & base_class,
    const std::set<std::string> & exclude_packages = {});

  /**
   * @brief Parse all associated plugin manifest files registered with the ament resource index and instantiate a
   * PluginClassLoader.
   *
   * This factory method additionally performs an ambiguity check. This means that it verifies that only unique class
   * names are being registered by the searched packages. Using the standard constructor, previously registered classes
   * are being overriden if the same class name is being parsed again. Therefore, this method offers the
   * preferred way of instantiating a PluginClassLoader.
   *
   * @tparam LoaderT Type of the plugin class loader.
   * @param base_package Name of the package containing the @p base_class. Will throw an error if it is not
   * installed.
   * @param base_class Fully qualified name of the base class that the class loader will use for determining which
   * classes listed in a plugins.xml file are associated with this instance.
   * @param exclude_packages Packages to exclude when searching for plugins.
   * @param reserved_names Map of reserved class names and the package name that makes the reservation. If any of these
   * class names are found, an error is being raised (Used internally during build time when installed resources aren't
   * available yet).
   * @throws ament_index_cpp::PackageNotFoundError if @p base_package is not installed.
   * @throws auto_apms_util::exceptions::ResourceError if no plugins of type @p base_class were found.
   * @throws auto_apms_util::exceptions::ResourceError if multiple packages register a resource using the same class
   * name (or a name in @p reserved_names is found).
   */
  static PluginClassLoader makeUnambiguousPluginClassLoader(
    const std::string & base_package, const std::string & base_class,
    const std::set<std::string> & exclude_packages = {},
    const std::map<std::string, std::string> & reserved_names = {});

  /**
   * @brief Retrieve a map that contains information about which package a plugin class belongs to.
   * @return Mapping according to {class_name: package_name}.
   */
  std::map<std::string, std::string> getClassPackageMap();
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

template <typename BaseT>
inline PluginClassLoader<BaseT>::PluginClassLoader(
  const std::string & base_package, const std::string & base_class, const std::set<std::string> & exclude_packages)
: pluginlib::ClassLoader<BaseT>(base_package, base_class, "", collectPluginXMLPaths(exclude_packages))
{
}

template <typename BaseT>
inline PluginClassLoader<BaseT> PluginClassLoader<BaseT>::makeUnambiguousPluginClassLoader(
  const std::string & base_package, const std::string & base_class, const std::set<std::string> & exclude_packages,
  const std::map<std::string, std::string> & reserved_names)
{
  std::map<std::string, std::vector<std::string>> packages_for_class_name;
  const std::set<std::string> packages =
    getPackagesWithResourceType(_AUTO_APMS_UTIL__RESOURCE_TYPE_NAME__PLUGINLIB, exclude_packages);
  for (const auto & package : packages) {
    auto single_package_loader =
      pluginlib::ClassLoader<BaseT>(base_package, base_class, "", {getPluginXMLPath(package)});
    for (const auto & class_name : single_package_loader.getDeclaredClasses()) {
      packages_for_class_name[class_name].push_back(package);
    }
  }

  // Reserved class names are considered as declared
  for (const auto & [class_name, package] : reserved_names) {
    packages_for_class_name[class_name].push_back(package + "(Build package)");
  }

  // Determine if there are duplicate class names
  std::vector<std::string> error_details;
  for (const auto & [class_name, packages] : packages_for_class_name) {
    if (packages.size() > 1) {
      error_details.push_back(
        "- Class '" + class_name + "' found in packages ['" + auto_apms_util::join(packages, "', '") + "'].");
    }
  }
  if (!error_details.empty()) {
    throw exceptions::ResourceError(
      "Ambiguous class names found! PluginClassLoader (Base: '" + base_class +
      "') created with makeUnambiguousPluginClassLoader() won't register resources from packages "
      "that use already existing lookup names. Found the following duplicates:\n" +
      auto_apms_util::join(error_details, "\n"));
  }
  return {base_package, base_class, exclude_packages};
}

template <typename BaseT>
inline std::map<std::string, std::string> PluginClassLoader<BaseT>::getClassPackageMap()
{
  std::map<std::string, std::string> map;
  for (const std::string & class_name : this->getDeclaredClasses()) {
    map[class_name] = this->getClassPackage(class_name);
  }
  return map;
}

}  // namespace auto_apms_util
