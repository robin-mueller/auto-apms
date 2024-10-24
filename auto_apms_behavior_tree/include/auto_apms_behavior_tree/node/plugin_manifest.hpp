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

#include "node_plugin_manifest_params.hpp"

namespace auto_apms_behavior_tree {

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Data structure for resource lookup data and configuration parameters required for loading and
 * registering a behavior tree node plugin.
 */
class BTNodePluginManifest
{
   public:
    /// Generated ROS2 parameter struct holding the core load information.
    using Params = node_plugin_manifest_params::Params::MapNames;

    /// Mapping of a node's name and its load parameters.
    using ParamMap = std::map<std::string, Params>;

    /// Listener for ROS2 node parameters.
    using ParamListener = node_plugin_manifest_params::ParamListener;

    static const std::string PARAM_NAME_NAMES;
    static const std::string PARAM_NAME_CLASS;
    static const std::string PARAM_NAME_PACKAGE;
    static const std::string PARAM_NAME_LIBRARY;
    static const std::string PARAM_NAME_PORT;
    static const std::string PARAM_NAME_WAIT_TIMEOUT;
    static const std::string PARAM_NAME_REQUEST_TIMEOUT;

    BTNodePluginManifest(const ParamMap& param_map = {});

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

    static BTNodePluginManifest FromParamListener(const ParamListener& param_listener);

    bool Contains(const std::string& node_name) const;

    Params& operator[](const std::string& node_name);
    const Params& operator[](const std::string& node_name) const;

    BTNodePluginManifest& Add(const std::string& node_name, const Params& p);

    BTNodePluginManifest& Remove(const std::string& node_name);

    BTNodePluginManifest& Merge(const BTNodePluginManifest& m);

    void ToFile(const std::string& file_path) const;

    std::string ToString() const;

    rcl_interfaces::msg::SetParametersResult ToROSParameters(rclcpp::Node::SharedPtr node_ptr,
                                                             const std::string& prefix = "") const;

    const ParamMap& map() const;

   private:
    ParamMap param_map_;
};

}  // namespace auto_apms_behavior_tree
