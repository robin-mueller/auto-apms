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

#include "px4_behavior/bt_factory.hpp"

#include <regex>
#include <set>

#include "class_loader/class_loader.hpp"
#include "px4_behavior/bt_node_registration.hpp"
#include "px4_behavior/get_resource.hpp"
#include "rcpputils/split.hpp"
#include "yaml-cpp/yaml.h"

namespace px4_behavior {

BT::RosNodeParams CreateRosNodeParams(const rclcpp::Node::SharedPtr& node,
                                      const std::string& default_port_value,
                                      const std::chrono::milliseconds& request_timeout,
                                      const std::chrono::milliseconds& wait_timeout)
{
    BT::RosNodeParams params;
    params.nh = node;
    params.default_port_value = default_port_value;
    params.server_timeout = request_timeout;
    params.wait_for_server_timeout = wait_timeout;
    return params;
}

bool RegisterBTNodePlugins(const rclcpp::Node::SharedPtr& node,
                           BT::BehaviorTreeFactory& factory,
                           const std::string& plugin_config_path,
                           const std::vector<std::string>& build_infos)
{
    return RegisterBTNodePlugins(node, factory, {plugin_config_path}, build_infos);
}

bool RegisterBTNodePlugins(const rclcpp::Node::SharedPtr& node,
                           BT::BehaviorTreeFactory& factory,
                           const std::set<std::string>& plugin_config_paths,
                           const std::vector<std::string>& build_infos)
{
    std::set<std::string> registered_plugins;
    std::map<std::string, std::unique_ptr<class_loader::ClassLoader>> loaders;

    for (const auto& plugin_config : plugin_config_paths) {
        YAML::Node root;
        try {
            root = YAML::LoadFile(plugin_config);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(),
                         "RegisterBTNodePlugins: Error loading config file '%s' - %s",
                         plugin_config.c_str(),
                         e.what());
            return false;
        }

        // Retrieve plugin library paths from build info of the current package (preferred)
        std::map<std::string, std::string> plugin_library_paths;
        for (const auto& build_info : build_infos) {
            std::vector<std::string> parts = rcpputils::split(build_info, '|');
            if (parts.size() != 2) { throw std::runtime_error("Invalid build info entry: " + build_info); }
            const std::string& classname = parts[0];
            const std::string& path = parts[1];
            plugin_library_paths[classname] = path;
        }

        for (YAML::const_iterator it = root.begin(); it != root.end(); ++it) {
            const auto package_name = it->first.as<std::string>();
            const auto plugins_node = root[package_name];

            // Add plugin library paths from ament_index resources if not already populated
            for (const auto& resource : FetchBTNodePluginResources(package_name)) {
                if (plugin_library_paths.find(resource.classname) == plugin_library_paths.end()) {
                    plugin_library_paths[resource.classname] = resource.library_path;
                }
            }

            for (YAML::const_iterator it = plugins_node.begin(); it != plugins_node.end(); ++it) {
                const std::string node_plugin_key = it->first.as<std::string>();

                std::vector<std::string> matching_classnames;
                for (const auto& r : plugin_library_paths) {
                    // Allow node_plugin_key to associate with the node's class name without explicitly stating the full
                    // namespace
                    if (std::smatch match;
                        std::regex_search(r.first, match, std::regex("(?:^|::)" + node_plugin_key + "$"))) {
                        matching_classnames.push_back(r.first);
                    }
                }
                if (matching_classnames.empty()) {
                    // Check fallback_plugin_paths if no matching resources are available
                    RCLCPP_ERROR(node->get_logger(),
                                 "RegisterBTNodePlugins: Couldn't find a behavior tree node plugin with key '%s' in "
                                 "package '%s'",
                                 node_plugin_key.c_str(),
                                 package_name.c_str());
                    return false;
                }
                if (matching_classnames.size() > 1) {
                    RCLCPP_ERROR(
                        node->get_logger(),
                        "RegisterBTNodePlugins: Cannot load plugin '%s' because multiple matching class names have "
                        "been found: %s\nIntroduce a unique namespace to the plugin key to resolve this issue.",
                        node_plugin_key.c_str(),
                        std::accumulate(matching_classnames.begin(),
                                        matching_classnames.end(),
                                        std::string(""),
                                        [](std::string& x, std::string& y) { return x.empty() ? y : x + ";" + y; })
                            .c_str());
                    return false;
                }

                const std::string& plugin_classname = matching_classnames[0];
                const std::string& plugin_library_path = plugin_library_paths[plugin_classname];

                // Check if the plugin has already been registered

                if (registered_plugins.find(plugin_classname) != registered_plugins.end()) continue;

                if (loaders.find(plugin_library_path) == loaders.end()) {
                    try {
                        loaders[plugin_library_path] = std::make_unique<class_loader::ClassLoader>(plugin_library_path);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(node->get_logger(),
                                     "RegisterBTNodePlugins: Failed to load library %s - %s",
                                     plugin_library_path.c_str(),
                                     e.what());
                        return false;
                    } catch (...) {
                        RCLCPP_ERROR(node->get_logger(),
                                     "RegisterBTNodePlugins: Failed to load library %s",
                                     plugin_library_path.c_str());
                        return false;
                    }
                }
                class_loader::ClassLoader* loader = loaders[plugin_library_path].get();

                // Look if the class we search for is actually present in the library.
                const std::string factory_classname =
                    "px4_behavior::BTNodeRegistrationTemplate<" + plugin_classname + ">";
                if (!loader->isClassAvailable<BTNodeRegistration>(factory_classname)) {
                    RCLCPP_ERROR(
                        node->get_logger(),
                        "RegisterBTNodePlugins: Class '%s' is not available in %s. You most likely misspelled the "
                        "class names of the corresponding behavior tree node in CMake when registering using "
                        "px4_behavior_register_plugins()",
                        factory_classname.c_str(),
                        loader->getLibraryPath().c_str());
                    return false;
                }

                auto plugin_instance = loader->createInstance<px4_behavior::BTNodeRegistration>(factory_classname);

                RCLCPP_DEBUG(node->get_logger(),
                             "RegisterBTNodePlugins: Register behavior tree node plugin '%s (%s)' from library %s",
                             node_plugin_key.c_str(),
                             plugin_classname.c_str(),
                             plugin_library_path.c_str());

                plugin_instance->RegisterForBehaviorTreeFactory(
                    factory,
                    node_plugin_key,
                    CreateRosNodeParams(node, it->second.IsNull() ? "" : it->second.as<std::string>()));

                // After successful plugin registration, mark it as registered
                registered_plugins.insert(plugin_classname);
            }
        }
    }
    return true;
}

BT::Tree CreateBehaviorTreeFromResource(const rclcpp::Node::SharedPtr& node,
                                        std::optional<const std::string> tree_file_name,
                                        std::optional<const std::string> tree_id,
                                        std::optional<const std::string> package_name)
{
    BT::BehaviorTreeFactory factory;
    auto parent_blackboard = BT::Blackboard::create();
    return CreateBehaviorTreeFromResource(node, factory, parent_blackboard, tree_file_name, tree_id, package_name);
}

BT::Tree CreateBehaviorTreeFromResource(const rclcpp::Node::SharedPtr& node,
                                        BT::BehaviorTreeFactory& factory,
                                        std::optional<const std::string> tree_file_name,
                                        std::optional<const std::string> tree_id,
                                        std::optional<const std::string> package_name)
{
    auto parent_blackboard = BT::Blackboard::create();
    return CreateBehaviorTreeFromResource(node, factory, parent_blackboard, tree_file_name, tree_id, package_name);
}

BT::Tree CreateBehaviorTreeFromResource(const rclcpp::Node::SharedPtr& node,
                                        const BT::Blackboard::Ptr& parent_blackboard,
                                        std::optional<const std::string> tree_file_name,
                                        std::optional<const std::string> tree_id,
                                        std::optional<const std::string> package_name)
{
    BT::BehaviorTreeFactory factory;
    return CreateBehaviorTreeFromResource(node, factory, parent_blackboard, tree_file_name, tree_id, package_name);
}

BT::Tree CreateBehaviorTreeFromResource(const rclcpp::Node::SharedPtr& node,
                                        BT::BehaviorTreeFactory& factory,
                                        const BT::Blackboard::Ptr& parent_blackboard,
                                        std::optional<const std::string> tree_file_name,
                                        std::optional<const std::string> tree_id,
                                        std::optional<const std::string> package_name)
{
    std::optional<BehaviorTreeResource> bt_resource_opt;
    try {
        bt_resource_opt = FetchBehaviorTreeResource(tree_file_name, tree_id, package_name);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(),
                     "CreateBehaviorTreeFromResource: Error fetching behavior tree resource for tree_file_name='%s', "
                     "tree_id='%s', package_name='%s': %s",
                     tree_file_name.value_or("").c_str(),
                     tree_id.value_or("").c_str(),
                     package_name.value_or("").c_str(),
                     e.what());
        return {};
    }
    if (!bt_resource_opt.has_value()) {
        RCLCPP_ERROR(node->get_logger(),
                     "CreateBehaviorTreeFromResource: No behavior tree resource found for tree_file_name='%s', "
                     "tree_id='%s', package_name='%s'",
                     tree_file_name.value_or("").c_str(),
                     tree_id.value_or("").c_str(),
                     package_name.value_or("").c_str());
        return {};
    }
    const BehaviorTreeResource& bt_resource = bt_resource_opt.value();

    // Regsiter associated node plugins
    if (!RegisterBTNodePlugins(node, factory, bt_resource.plugin_config_paths)) {
        RCLCPP_ERROR(node->get_logger(),
                     "CreateBehaviorTreeFromResource: Registering behavior tree node plugins failed");
        return {};
    }

    // Register behavior tree resource file
    try {
        factory.registerBehaviorTreeFromFile(bt_resource.tree_path);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(),
                     "CreateBehaviorTreeFromResource: Error registering behavior tree resource file %s: %s",
                     bt_resource.tree_path.c_str(),
                     e.what());
        return {};
    }

    // Try to create tree from factory
    BT::Tree tree;
    const std::string& tid = tree_id.value_or("");
    try {
        tree = factory.createTree(tid, parent_blackboard);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(),
                     "CreateBehaviorTreeFromResource: Error creating tree with ID %s from file %s: %s",
                     tid.c_str(),
                     bt_resource.tree_path.c_str(),
                     e.what());
        return {};
    }

    return tree;
}

}  // namespace px4_behavior
