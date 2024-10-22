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

#include "auto_apms_behavior_tree/node/plugin_loader.hpp"

#include <algorithm>
#include <fstream>

#include "auto_apms_behavior_tree/exceptions.hpp"
#include "auto_apms_core/resources.hpp"
#include "rcpputils/split.hpp"

namespace auto_apms_behavior_tree {

BTNodePluginLoader::BTNodePluginLoader(rclcpp::Node::SharedPtr node_ptr, const std::set<std::string>& package_names)
    : ClassLoader{"auto_apms_behavior_tree",
                  "auto_apms_behavior_tree::BTNodePluginBase",
                  "",
                  GetPluginXMLFilePaths(package_names.empty() ? auto_apms_core::GetAllPackagesWithResource(
                                                                    _AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__NODE)
                                                              : package_names)},
      node_ptr_{node_ptr},
      param_prefix_{"node_plugins."},
      param_listener_{node_ptr_, param_prefix_}
{}

std::vector<std::string> BTNodePluginLoader::GetPluginXMLFilePaths(const std::set<std::string>& package_names)
{
    std::vector<std::string> xml_paths;
    for (const auto& package_name : package_names) {
        std::string content;
        std::string base_path;
        if (ament_index_cpp::get_resource(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__NODE,
                                          package_name,
                                          content,
                                          &base_path)) {
            std::vector<std::string> paths = rcpputils::split(content, '\n', true);
            if (paths.size() != 1) {
                throw std::runtime_error("Invalid behavior tree node plugin resource file (Package: '" + package_name +
                                         "').");
            }
            xml_paths.push_back(base_path + '/' + paths[0]);
        }
    }
    return xml_paths;
}

void BTNodePluginLoader::AutoCompleteManifest(Manifest& manifest)
{
    for (const auto& [node_name, params] : manifest.map()) {
        const std::string node_package_name = getClassPackage(params.class_name);
        if (node_package_name.empty()) {
            throw exceptions::ResourceNotFoundError(
                "Cannot find class '" + params.class_name + "' required by node '" + node_name +
                "', because no such resource is registered with this plugin loader instance.");
        }
        const std::string node_lib_path = getClassLibraryPath(params.class_name);
        if (!params.package.empty()) {
            // Verify the plugin can be found in the package
            if (params.package == node_package_name) {
                throw exceptions::ResourceNotFoundError(
                    "Cannot find class '" + params.class_name + "' required by node '" + node_name + "' in package '" +
                    params.package + "'. Internally, the resource is registered by package '" + node_package_name +
                    "' instead. This can occur if multiple packages register a node plugin with the same class name. "
                    "To resolve this issue, introduce a unique package namespace to the respective class names.");
            }
        }
        manifest[node_name].package = node_package_name;
        manifest[node_name].library = node_lib_path;  // Any entry in library will be overwritten
    }
}

void BTNodePluginLoader::Load(const Manifest& manifest, BT::BehaviorTreeFactory& factory)
{
    for (const auto& [node_name, params] : manifest.map()) {
        // Check if the class we search for is actually available with the loader.
        if (!isClassAvailable(params.class_name)) {
            throw exceptions::BTNodePluginLoadingError{
                "Node '" + node_name + " (" + params.class_name +
                ")' cannot be loaded, because it's not registered by the packages searched by the plugin loader. It's "
                "also possible that you misspelled the class name in CMake when registering it in the CMakeLists.txt "
                "using auto_apms_behavior_tree_register_nodes()."};
        }

        RCLCPP_DEBUG(node_ptr_->get_logger(),
                     "BTNodePluginLoader::Load - Register behavior tree node plugin '%s (%s)' from library %s.",
                     node_name.c_str(),
                     params.class_name.c_str(),
                     getClassLibraryPath(params.class_name).c_str());

        pluginlib::UniquePtr<BTNodePluginBase> plugin_instance;
        try {
            plugin_instance = createUniqueInstance(params.class_name);
        } catch (const std::exception& e) {
            throw exceptions::BTNodePluginLoadingError(
                "Failed to create an instance of node '" + node_name + " (" + params.class_name +
                ")'. Remember that the AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE macro must be called in the source file "
                "for the plugin to be discoverable. Error message: " +
                e.what() + ".");
        }

        try {
            if (plugin_instance->RequiresROSNodeParams()) {
                RosNodeParams ros_params;
                ros_params.nh = node_ptr_;
                ros_params.default_port_name = params.port;
                ros_params.wait_for_server_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::duration<double>(params.wait_timeout));
                ros_params.request_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::duration<double>(params.request_timeout));
                plugin_instance->RegisterWithBehaviorTreeFactory(factory, node_name, &ros_params);
            }
            else {
                plugin_instance->RegisterWithBehaviorTreeFactory(factory, node_name);
            }
        } catch (const std::exception& e) {
            throw exceptions::BTNodePluginLoadingError("Failed to register node '" + node_name + " (" +
                                                       params.class_name + ")' with factory: " + e.what() + ".");
        }
    }
}

void BTNodePluginLoader::Load(BT::BehaviorTreeFactory& factory) { Load(GetManifestFromParameters(), factory); }

BTNodePluginLoader::Manifest BTNodePluginLoader::GetManifestFromParameters()
{
    return param_listener_.get_params().names_map;
}

void BTNodePluginLoader::UpdateParameters(const Manifest& manifest)
{
    auto m = manifest;
    AutoCompleteManifest(m);
    m.ToROSParameters(node_ptr_, param_prefix_);
}

}  // namespace auto_apms_behavior_tree
