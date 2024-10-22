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

#include "auto_apms_behavior_tree/node/plugin_base.hpp"
#include "auto_apms_behavior_tree/node/plugin_manifest.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/node.hpp"

namespace auto_apms_behavior_tree {

/**
 * @brief Utility class that manages the process of loading and registering behavior tree node plugins.
 * @ingroup auto_apms_behavior_tree
 */
class BTNodePluginLoader : private pluginlib::ClassLoader<BTNodePluginBase>
{
    using ManifestParamListener = node_plugin_loader_params::ParamListener;

   public:
    using Manifest = BTNodePluginManifest;

    BTNodePluginLoader(rclcpp::Node::SharedPtr node_ptr, const std::set<std::string>& package_names = {});

    /**
     * @brief Load behavior tree node plugins and register with behavior tree factory.
     *
     * @param[in] manifest Parameters used for loading and configuring the behavior tree node.
     * @param[in,out] factory Behavior tree factory instance that the behavior tree nodes will register with.
     * @throw exceptions::BTNodePluginLoadingError if registration fails.
     */
    void Load(const Manifest& manifest, BT::BehaviorTreeFactory& factory);

    /**
     * @overload
     *
     * Infers the manifest for loading from the node's parameters.
     */
    void Load(BT::BehaviorTreeFactory& factory);

    /**
     * @brief Fill the node plugin resource information in @p manifest.
     *
     * This function autocompletes the package and library fields of the manifest according to this instance's
     * underlying resource information if possible. Depending on the package paramters, the manifest will be updated:
     *
     * - **Package undefined**: The library path will be resolved by looking up the class name
     * in the internal map of node plugin resources **considering all packages specified in the constructor**.
     *
     * - **Package defined**: The library path will be resolved by looking up the class name in
     * the internal map of node plugin resources **considering only resources registered by the given package**.
     *
     * The library parameter will be overwritten in any case and other parameters remain untouched and are simply
     * copied.
     *
     * This method is mainly used for introspection purposes.
     *
     * @param manifest Manifest object to be autocompleted.
     * @throw exceptions::ResourceNotFoundError if no unique resource for a node can be found.
     */
    void AutoCompleteManifest(Manifest& manifest);

    Manifest GetManifestFromParameters();

    void UpdateParameters(const Manifest& manifest);

   private:
    static std::vector<std::string> GetPluginXMLFilePaths(const std::set<std::string>& package_names);

    rclcpp::Node::SharedPtr node_ptr_;
    const std::string param_prefix_;
    ManifestParamListener param_listener_;
};

}  // namespace auto_apms_behavior_tree