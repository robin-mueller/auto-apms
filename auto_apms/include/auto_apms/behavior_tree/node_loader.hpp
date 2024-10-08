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

#include "auto_apms/behavior_tree/node_plugin_load_manifest.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "rclcpp/node.hpp"

namespace auto_apms {

class BTNodePluginLoader
{
   public:
    using Manifest = detail::BTNodePluginLoadManifest;

    BTNodePluginLoader(rclcpp::Node::SharedPtr node_ptr, const Manifest& manifest);

    /**
     * @brief Load behavior tree node plugins and register with behavior tree factory.
     *
     * @param[in] node_ptr ROS node to use for ROS specific behavior tree nodes.
     * @param[in] manifest Parameters used for loading and configuring the behavior tree node.
     * @param[in,out] factory Behavior tree factory instance that the behavior tree nodes will register with.
     * @param[in,out] class_loader The class loader to use for loading the shared libraries.
     * @throw exceptions::BTNodePluginLoadingError if registration fails.
     */
    static void Load(rclcpp::Node::SharedPtr node_ptr,
                     const Manifest& manifest,
                     BT::BehaviorTreeFactory& factory,
                     class_loader::MultiLibraryClassLoader& class_loader);

    /// @overload
    void Load(BT::BehaviorTreeFactory& factory);

   private:
    rclcpp::Node::SharedPtr node_ptr_;
    Manifest manifest_;
    std::unique_ptr<class_loader::MultiLibraryClassLoader> class_loader_ptr_;
};

}  // namespace auto_apms