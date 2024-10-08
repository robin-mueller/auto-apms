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

#include "auto_apms/behavior_tree/node_loader.hpp"

#include <algorithm>
#include <fstream>

#include "auto_apms/behavior_tree/node_plugin_base.hpp"
#include "auto_apms/exceptions.hpp"

namespace auto_apms {

BTNodePluginLoader::BTNodePluginLoader(rclcpp::Node::SharedPtr node_ptr, const Manifest& manifest)
    : node_ptr_{node_ptr},
      manifest_{manifest},
      class_loader_ptr_{std::make_unique<class_loader::MultiLibraryClassLoader>(false)}
{}

void BTNodePluginLoader::Load(rclcpp::Node::SharedPtr node_ptr,
                              const Manifest& manifest,
                              BT::BehaviorTreeFactory& factory,
                              class_loader::MultiLibraryClassLoader& class_loader)
{
    const auto verified_manifest = manifest.Verify();
    for (const auto& [node_name, params] : verified_manifest.MapView()) {
        if (!params.library.has_value()) {
            throw exceptions::BTNodePluginLoadingError("Load parameters for node '" + node_name +
                                                       "' do not specify a library path.");
        }
        const auto& library_path = params.library.value();

        if (!class_loader.isLibraryAvailable(library_path)) {
            try {
                class_loader.loadLibrary(library_path);
            } catch (const std::exception& e) {
                throw exceptions::BTNodePluginLoadingError("Failed to load library '" + library_path +
                                                           "': " + e.what() + ".");
            }
        }

        // Look if the class we search for is actually present in the library.
        const std::string factory_classname = "auto_apms::detail::BTNodePlugin<" + params.class_name + ">";
        const auto classes = class_loader.getAvailableClassesForLibrary<detail::BTNodePluginBase>(library_path);
        if (std::find(classes.begin(), classes.end(), factory_classname) == classes.end()) {
            throw exceptions::BTNodePluginLoadingError{
                "Node '" + node_name + " (" + params.class_name + ")' cannot be loaded, because factory class '" +
                factory_classname +
                "' couldn't be found. You most likely misspelled the class name in CMake when registering it using "
                "auto_apms_register_plugins() or forgot to call the AUTO_APMS_REGISTER_BEHAVIOR_TREE_NODE macro in the "
                "source file."};
        }

        RCLCPP_DEBUG(node_ptr->get_logger(),
                     "BTNodePluginLoader::Load - Register behavior tree node plugin '%s (%s)' from library %s.",
                     node_name.c_str(),
                     params.class_name.c_str(),
                     library_path.c_str());

        auto plugin_instance = class_loader.createInstance<detail::BTNodePluginBase>(factory_classname, library_path);
        try {
            if (plugin_instance->RequiresROSNodeParams()) {
                const auto ros_node_params = params.CreateROSNodeParams(node_ptr);
                plugin_instance->RegisterWithBehaviorTreeFactory(factory, node_name, &ros_node_params);
            }
            else {
                if (params.IsROSSpecific()) {
                    RCLCPP_WARN(
                        node_ptr->get_logger(),
                        "BTNodePluginLoader::Load - ROS specific parameters were given for node '%s (%s)', but it "
                        "doesn't require any.",
                        node_name.c_str(),
                        params.class_name.c_str());
                }
                plugin_instance->RegisterWithBehaviorTreeFactory(factory, node_name);
            }
        } catch (const std::exception& e) {
            throw exceptions::BTNodePluginLoadingError("Failed to register node '" + node_name + " (" +
                                                       params.class_name + ")': " + e.what() + ".");
        }
    }
}

void BTNodePluginLoader::Load(BT::BehaviorTreeFactory& factory)
{
    Load(node_ptr_, manifest_, factory, *class_loader_ptr_);
}

}  // namespace auto_apms
