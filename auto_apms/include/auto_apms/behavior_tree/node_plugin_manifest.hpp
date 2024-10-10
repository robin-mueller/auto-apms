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

#include "auto_apms/version.hpp"

#pragma once

#include "auto_apms/behavior_tree/resources.hpp"
#include "auto_apms_rosparam__bt_node_plugin_manifest.hpp"

namespace auto_apms::detail {

/**
 * @brief Resource lookup data and configuration parameters required for loading and registering a behavior tree node
 * plugin.
 */
class BTNodePluginManifest
{
   public:
    /// @brief Generated ROS2 parameter struct holding the core load information.
    using Params = auto_apms::detail::rosparam::bt_node_plugin_manifest::Params::MapNames;

    /// @brief Mapping of a node's name and its load parameters.
    using ParamMap = std::map<std::string, Params>;

    static const std::string PARAM_NAME_NAMES;
    static const std::string PARAM_NAME_CLASS;
    static const std::string PARAM_NAME_PACKAGE;
    static const std::string PARAM_NAME_LIBRARY;
    static const std::string PARAM_NAME_PORT;
    static const std::string PARAM_NAME_REQUEST_TIMEOUT;
    static const std::string PARAM_NAME_WAIT_TIMEOUT;

    BTNodePluginManifest(const ParamMap& param_map = {});

    static BTNodePluginManifest FromResource(const BehaviorTreeResource& resource);

    /**
     * @brief Create a node plugin manifest from multiple files. They are loaded in the given order.
     * @param file_paths Paths to the manifest files.
     */
    static BTNodePluginManifest FromFiles(const std::vector<std::string>& file_paths);

    /**
     * @brief Create a node plugin manifest from a file.
     * @param file_path Path to the manifest file.
     */
    static BTNodePluginManifest FromFile(const std::string& file_path);

    static BTNodePluginManifest Parse(const std::string& manifest_str);

    bool Contains(const std::string& node_name) const;

    Params& operator[](const std::string& node_name);
    const Params& operator[](const std::string& node_name) const;

    BTNodePluginManifest& Add(const std::string& node_name, const Params& p);

    BTNodePluginManifest& Remove(const std::string& node_name);

    BTNodePluginManifest& Merge(const BTNodePluginManifest& m);

    /**
     * @brief Find/Verify the shared library paths of the plugins specified in the manifest.
     *
     * The behavior of this function can be summarized according to the following ruleset:
     * - **Library undefined** - **Package undefined**: The library path will be resolved by looking up the class name
     * in the installed node plugin resources. There must be exactly one package associated with that class name.
     * - **Library undefined** - **Package defined**: The library path will be resolved by looking up the class name in
     * the resources registered by the given package.
     * - **Library defined** - **Package undefined**: The given library path will be left as is and no further
     * validation will be conducted, so you should know what you're doing.
     * - **Library defined** - **Package defined**: The given package will be searched for a matching resource and it
     * will be verified that the library associated with the resource matches the one specified.
     *
     * So this function checks each plugin's configuration parameters and adds the absolute paths of associated
     * libraries to the returned manifest if not already specified. In case the library is specified and a package is
     * given, a verification is performed. Otherwise, the library entry will be left as is. The other parameters remain
     * untouched and are simply copied.
     *
     * This should be called before loading the plugins of the manifest to ensure that the correct plugins will be
     * found.
     *
     * @param ignore_packages Package names to ignore when searching for resources.
     * @throw exceptions::ResourceNotFoundError if the corresponding required resource cannot be determined.
     * @throw exceptions::BTNodePluginManifestError if a violation of the above mentioned rules occurs.
     * @return New manifest object with updated parameters.
     */
    BTNodePluginManifest& LocateAndVerifyLibraries(const std::set<std::string> ignore_packages = {});

    void ToFile(const std::string& file_path) const;

    std::string ToString() const;

    void ToROSParameters(rclcpp::Node::SharedPtr node_ptr, const std::string& prefix = "") const;

    const ParamMap& map() const;

   private:
    ParamMap param_map_;
};

}  // namespace auto_apms::detail
