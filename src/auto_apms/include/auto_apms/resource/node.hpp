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
#include <set>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "class_loader/class_loader.hpp"
#include "rclcpp/node.hpp"
#include "yaml-cpp/node/convert.h"

namespace auto_apms {
namespace resource {

/**
 * @brief Resource and configuration information required for registering a behavior tree node plugin
 *
 * Some parameters are optional and there are certain rules for interpreting different combinations of parameters.
 *
 * @sa ValidateBTNodeManifest
 */
struct BTNodeManifest
{
    // Required
    std::string class_name;  ///< Name of the behavior tree node class that will eventually be loaded and registered

    // Optional
    std::optional<std::string> package;     ///< Name of the package where the corresponding resource can be found
    std::optional<std::string> library;     ///< Path to the shared library that defines the node class
    std::optional<std::string> port;        ///< ROS topic or action name
    std::optional<double> wait_timeout;     ///< Timeout [s] for initially discovering the server
    std::optional<double> request_timeout;  ///< Timeout [s] for waiting for a goal response

    /**
     * @brief Determine if any parameters specifically intended to be passed to ROS behavior tree nodes on construction
     * have been specified.
     * @return `true` if that's the case, `false` otherwise.
     */
    bool IsROSSpecific() const;

    /**
     * @brief Create BT::RosNodeParams from parameters specified by the manifest.
     * @param node_ptr ROS node to be forwarded.
     * @return BT::RosNodeParams that comply with the manifest's parameters.
     */
    BT::RosNodeParams CreateROSNodeParams(rclcpp::Node::SharedPtr node_ptr) const;

    /**
     * @brief Get all required parameter names.
     * @return Set of names.
     */
    static std::set<std::string> GetRequiredNames();
};

/**
 * @brief Map associating a node's manifest with it's registration name.
 *
 * According to {<node_name>: <manifest>}.
 */
using BTNodeManifestMap = std::map<std::string, BTNodeManifest>;

/// @brief Struct for behavior tree node plugin resource data
struct BTNodeResource
{
    std::string class_name;    ///< Name of the class that can be loaded from BTNodeResource::library_path
    std::string library_path;  ///< Path to the library associated with this resource
};

/**
 * @brief Collect all behavior tree node plugin resources registered by a certain package.
 * @param package_name Name of the package to search for resources.
 * @return Collection of all resources found in @p package_name.
 */
std::vector<BTNodeResource> FetchBTNodeResources(const std::string& package_name);

/**
 * @brief Load and parse node plugin manifest files and store its contents in a BTNodeManifestMap.
 *
 * Parsing is done by converting a YAML::Node to a BTNodeManifest using the corresponding conversion specialization
 * YAML::convert<auto_apms::resource::BTNodeManifestMap>.
 *
 * @param paths Paths to the manifest files. They are parsed in the given order.
 * @return Map containing a BTNodeManifest object for each node that has been specified within the files.
 * @throw std::runtime_error if a single node is mentioned multiple times.
 * @throw std::runtime_error if not all required parameters have been specified.
 */
BTNodeManifestMap ParseBTNodeManifestFile(const std::vector<std::string>& paths);

/// @overload
BTNodeManifestMap ParseBTNodeManifestFile(const std::string& path);

/**
 * @brief Validate the node manifest and infer required information from resources.
 *
 * This function checks the integrity of each node manifest given in @p manifest_map to ensure that following this step,
 * ::RegisterBTNodePlugins will successfully register the nodes specified in @p manifest_map.
 *
 * This is done by inspecting the combination of parameters BTNodeManifest::class_name, BTNodeManifest::library and
 * BTNodeManifest::package according to the following logic:
 * - **Library undefined** - **Package undefined**: The library path will be resolved by looking up the class name in
 * the installed node plugin resources. There must be exactly one package associated with that class name.
 * - **Library undefined** - **Package defined**: The library path will be resolved by looking up the class name in the
 * resources registered by the given package.
 * - **Library defined** - **Package undefined**: The given library path will be left as is (you should know what you're
 * doing).
 * - **Library defined** - **Package defined**: It will be verified that the package has registered that the provided
 * class name is indeed associated with the given library.
 *
 * Other parameters are not validated and left as is.
 *
 * @param manifest_map Node manifests to be validated.
 * @return Validated manifest map basically copied from @p manifest_map, but certain parameters have been set according
 * to the rules above.
 */
BTNodeManifestMap ValidateBTNodeManifest(const BTNodeManifestMap& manifest_map);

/**
 * @overload
 * @param manifest_path Path to a node manifest file.
 * @return Map of the node manifests found in @p manifest_path.
 */
BTNodeManifestMap ValidateBTNodeManifest(const std::string& manifest_path);

/**
 * @overload
 * @param manifest_paths Multiple paths to node manifest files. They are parsed in the given order.
 * @return Concatenated map of node manifests found in @p manifest_paths.
 */
BTNodeManifestMap ValidateBTNodeManifest(const std::vector<std::string>& manifest_paths);

}  // namespace resource

/**
 * @brief Register behavior tree node plugins with an instance of BT::BehaviorTreeFactory.
 *
 * With this function signature, it is obligatory to call resource::ValidateBTNodeManifest with @p manifest_map or
 * manually specify a library for each node plugin to make sure that the registration succeeds. Alternatively, you can
 * use the overloads that accept one or more manifest files. These will automatically parse and validate the each node's
 * manifest for you.
 *
 * @param[in] node_ptr ROS node to use for ROS specific behavior tree nodes.
 * @param[in] manifest_map Map of behavior tree node manifests.
 * @param[out] factory Behavior tree factory instance that the behavior tree nodes will register with.
 * @return `true` if registration was successfull, `false` otherwise.
 */
bool RegisterBTNodePlugins(rclcpp::Node::SharedPtr node_ptr,
                           const resource::BTNodeManifestMap& manifest_map,
                           BT::BehaviorTreeFactory& factory);

/**
 * @overload
 * @param manifest_path Path to a node manifest file.
 */
bool RegisterBTNodePlugins(rclcpp::Node::SharedPtr node_ptr,
                           const std::string& manifest_path,
                           BT::BehaviorTreeFactory& factory);

/**
 * @overload
 * @param manifest_paths Multiple paths to node manifest files. They are parsed in the given order.
 */
bool RegisterBTNodePlugins(rclcpp::Node::SharedPtr node_ptr,
                           const std::vector<std::string>& manifest_paths,
                           BT::BehaviorTreeFactory& factory);

}  // namespace auto_apms

namespace YAML {

template <>
struct convert<auto_apms::resource::BTNodeManifestMap>
{
    static Node encode(const auto_apms::resource::BTNodeManifestMap& rhs);
    static bool decode(const Node& node, auto_apms::resource::BTNodeManifestMap& lhs);
};

}  // namespace YAML
