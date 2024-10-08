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
#include <optional>

#include "behaviortree_ros2/ros_node_params.hpp"

namespace auto_apms::detail {

class BTNodePluginLoadManifest
{
   public:
    static constexpr char YAML_PARAM_CLASS[] = "class";
    static constexpr char YAML_PARAM_PACKAGE[] = "package";
    static constexpr char YAML_PARAM_LIBRARY[] = "library";
    static constexpr char YAML_PARAM_PORT[] = "port";
    static constexpr char YAML_PARAM_REQUEST_TIMEOUT[] = "request_timeout";
    static constexpr char YAML_PARAM_WAIT_TIMEOUT[] = "wait_timeout";

    /**
     * @brief Resource lookup data and configuration parameters required for registering a behavior tree node plugin.
     *
     * It's not necessary to specify all of the optional members, but you have to stick to the rules specified in
     * ::ValidateBTNodeRegistrationManifest. The function infers additional information based on the provided value
     * combination.
     */
    struct Params
    {
        // Default values
        static constexpr double WAIT_TIMEOUT_DEFAULT_SEC = 3.0;     ///< Default value for wait_timeout
        static constexpr double REQUEST_TIMEOUT_DEFAULT_SEC = 1.5;  ///< Default value for request_timeout

        // Required
        std::string class_name;  ///< Name of the behavior tree node class that will eventually be loaded and registered

        // Optional
        std::optional<std::string> package;     ///< Name of the package where the corresponding resource can be found
        std::optional<std::string> library;     ///< Path to the shared library that defines the node class
        std::optional<std::string> port;        ///< ROS topic or action name
        std::optional<double> wait_timeout;     ///< Timeout [s] for initially discovering the server
        std::optional<double> request_timeout;  ///< Timeout [s] for waiting for a goal response

        /**
         * @brief Determine if any parameters specifically intended to be passed to ROS behavior tree nodes on
         * construction have been specified.
         * @return `true` if that's the case, `false` otherwise.
         */
        bool IsROSSpecific() const;

        /**
         * @brief Create BT::RosNodeParams from parameters specified by the manifest.
         * @param node_ptr ROS node to be forwarded.
         * @return BT::RosNodeParams that comply with the manifest's parameters.
         */
        BT::RosNodeParams CreateROSNodeParams(rclcpp::Node::SharedPtr node_ptr) const;
    };

   protected:
    /// @brief Mapping of a node's name with its load configuration.
    using ParamMap = std::map<std::string, Params>;

   public:
    BTNodePluginLoadManifest() = default;

    BTNodePluginLoadManifest(const std::vector<std::pair<std::string, Params>> values);

    static BTNodePluginLoadManifest Parse(const std::string& manifest_str);

    /**
     * @brief Create a node plugin manifest from multiple files. They are loaded in the given order.
     * @param file_paths Paths to the manifest files.
     */
    static BTNodePluginLoadManifest FromFiles(const std::vector<std::string>& file_paths);

    /**
     * @brief Create a node plugin manifest from a file.
     * @param file_path Path to the manifest file.
     */
    static BTNodePluginLoadManifest FromFile(const std::string& file_path);

    bool Contains(const std::string& node_name) const;
    const ParamMap& MapView() const;

    Params& operator[](const std::string& node_name);
    const Params& operator[](const std::string& node_name) const;

    void Add(const std::string& node_name, const Params& p);

    void Remove(const std::string& node_name);

    void Merge(const BTNodePluginLoadManifest& m);

    /**
     * @brief Verify the given load configuration.
     *
     * This function checks the integrity of a BT node plugin load configuration specified by @p config to ensure that
     * loading the plugin is successful.
     *
     * This is done by evaluating the combination of the parameters for the class name, the library path and the lookup
     * package according to the following logic:
     * - **Library undefined** - **Package undefined**: The library path will be resolved by looking up the class name
     * in the installed node plugin resources. There must be exactly one package associated with that class name.
     * - **Library undefined** - **Package defined**: The library path will be resolved by looking up the class name in
     * the resources registered by the given package.
     * - **Library defined** - **Package undefined**: The given library path will be left as is and no further
     * validation will be conducted, so you should know what you're doing.
     * - **Library defined** - **Package defined**: It will be verified that the given package has indeed registered a
     * resource that associates the class name with the given library.
     *
     * Other parameters are left as is.
     *
     * @param params Load configuration object.
     * @param ignore_packages Package names to ignore when searching for resources.
     * @throw exceptions::ResourceNotFoundError if a required resource is not registered.
     * @throw exceptions::BTNodePluginManifestLogicError if verification fails due to a parameter logic error.
     */
    static Params VerifyParameters(const Params& params, const std::set<std::string> ignore_packages = {});

    BTNodePluginLoadManifest Verify() const;

    void ToFile(const std::string& file_path) const;

    std::string ToString() const;

   protected:
    ParamMap param_map_;
};

}  // namespace auto_apms::detail
