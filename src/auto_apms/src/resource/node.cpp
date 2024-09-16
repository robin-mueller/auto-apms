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

#include "auto_apms/resource/node.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_resource.hpp"
#include "ament_index_cpp/get_resources.hpp"
#include "auto_apms/bt_node_registrar.hpp"
#include "rcpputils/split.hpp"
#include "yaml-cpp/yaml.h"

#define BT_NODE_YAML_PARAM_CLASS "class"
#define BT_NODE_YAML_PARAM_PORT "port"
#define BT_NODE_YAML_PARAM_PACKAGE "package"
#define BT_NODE_YAML_PARAM_LIBRARY "library"
#define BT_NODE_YAML_PARAM_REQUEST_TIMEOUT "request_timeout"
#define BT_NODE_YAML_PARAM_WAIT_TIMEOUT "wait_timeout"

namespace YAML {

Node convert<auto_apms::resource::BTNodeRegistrationConfigMap>::encode(
    const auto_apms::resource::BTNodeRegistrationConfigMap& rhs)
{
    Node node;
    for (const auto& it : rhs) {
        Node param_node;
        const auto& obj = it.second;

        // Required parameters
        param_node[BT_NODE_YAML_PARAM_CLASS] = obj.class_name;

        // Optional parameters
        if (obj.port.has_value()) param_node[BT_NODE_YAML_PARAM_PORT] = obj.port.value();
        if (obj.package.has_value()) param_node[BT_NODE_YAML_PARAM_PACKAGE] = obj.package.value();
        if (obj.library.has_value()) param_node[BT_NODE_YAML_PARAM_LIBRARY] = obj.library.value();
        if (obj.request_timeout.has_value())
            param_node[BT_NODE_YAML_PARAM_REQUEST_TIMEOUT] = obj.request_timeout.value();
        if (obj.wait_timeout.has_value()) param_node[BT_NODE_YAML_PARAM_WAIT_TIMEOUT] = obj.wait_timeout.value();

        node[it.first] = param_node;
    }
    return node;
}

bool convert<auto_apms::resource::BTNodeRegistrationConfigMap>::decode(
    const Node& node, auto_apms::resource::BTNodeRegistrationConfigMap& lhs)
{
    const std::map<std::string, NodeType::value> param_type_map;

    if (!node.IsMap()) throw std::runtime_error("Root node must be map. Cannot convert to BTNodeRegistrationConfigMap");
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
        const auto& name = it->first.as<std::string>();
        const auto& params_node = it->second;
        if (!params_node.IsMap())
            throw std::runtime_error("Params node must be map. Cannot convert to BTNodeRegistrationConfigMap");

        std::set<std::string> all_param_names;
        auto_apms::resource::BTNodeRegistrationConfig obj;
        for (YAML::const_iterator p = params_node.begin(); p != params_node.end(); ++p) {
            const auto param_key = p->first.as<std::string>();
            const auto& val = p->second;
            all_param_names.insert(param_key);

            if (param_key == BT_NODE_YAML_PARAM_CLASS) {
                if (!val.IsScalar())
                    throw std::runtime_error("Key '" + std::string(BT_NODE_YAML_PARAM_CLASS) +
                                             "' is not scalar. Cannot convert to BTNodeRegistrationConfigMap");
                obj.class_name = val.as<std::string>();
                continue;
            }

            if (param_key == BT_NODE_YAML_PARAM_PORT) {
                if (!val.IsScalar())
                    throw std::runtime_error("Key '" + std::string(BT_NODE_YAML_PARAM_PORT) +
                                             "' is not scalar. Cannot convert to BTNodeRegistrationConfigMap");
                obj.port = val.as<std::string>();
                continue;
            }

            if (param_key == BT_NODE_YAML_PARAM_PACKAGE) {
                if (!val.IsScalar())
                    throw std::runtime_error("Key '" + std::string(BT_NODE_YAML_PARAM_PACKAGE) +
                                             "' is not scalar. Cannot convert to BTNodeRegistrationConfigMap");
                obj.package = val.as<std::string>();
                continue;
            }

            if (param_key == BT_NODE_YAML_PARAM_LIBRARY) {
                if (!val.IsScalar())
                    throw std::runtime_error("Key '" + std::string(BT_NODE_YAML_PARAM_LIBRARY) +
                                             "' is not scalar. Cannot convert to BTNodeRegistrationConfigMap");
                obj.library = val.as<std::string>();
                continue;
            }

            if (param_key == BT_NODE_YAML_PARAM_WAIT_TIMEOUT) {
                if (!val.IsScalar())
                    throw std::runtime_error("Key '" + std::string(BT_NODE_YAML_PARAM_WAIT_TIMEOUT) +
                                             "' is not scalar. Cannot convert to BTNodeRegistrationConfigMap");
                obj.wait_timeout = val.as<double>();
                continue;
            }

            if (param_key == BT_NODE_YAML_PARAM_REQUEST_TIMEOUT) {
                if (!val.IsScalar())
                    throw std::runtime_error("Key '" + std::string(BT_NODE_YAML_PARAM_REQUEST_TIMEOUT) +
                                             "' is not scalar. Cannot convert to BTNodeRegistrationConfigMap");
                obj.request_timeout = val.as<double>();
                continue;
            }

            // Unkown parameter
            throw std::runtime_error("Unkown parameter '" + param_key +
                                     "'. Cannot convert to BTNodeRegistrationConfigMap");
        }

        // Check if all required parameters are specified
        const auto required_names = auto_apms::resource::BTNodeRegistrationConfig::GetRequiredNames();
        if (!std::includes(all_param_names.begin(),
                           all_param_names.end(),
                           required_names.begin(),
                           required_names.end()))
            std::runtime_error(
                "Registration configuration for node '" + name +
                "' is missing at least one required parameter. Cannot convert to BTNodeRegistrationConfigMap");

        lhs[name] = obj;
    }

    return true;
}

}  // namespace YAML

namespace auto_apms {
namespace resource {

bool BTNodeRegistrationConfig::IsROSSpecific() const
{
    return port.has_value() || wait_timeout.has_value() || request_timeout.has_value();
}

BT::RosNodeParams BTNodeRegistrationConfig::CreateROSNodeParams(rclcpp::Node::SharedPtr node_ptr) const
{
    auto wait_timeout_default_sec = 3.0;
    auto request_timeout_default_sec = 1.5;

    BT::RosNodeParams params;
    params.nh = node_ptr;
    params.default_port_value = port.value_or("");
    params.wait_for_server_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(wait_timeout.value_or(wait_timeout_default_sec)));
    params.server_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(request_timeout.value_or(request_timeout_default_sec)));
    return params;
}

std::set<std::string> BTNodeRegistrationConfig::GetRequiredNames() { return {BT_NODE_YAML_PARAM_CLASS}; }

BTNodeRegistrationConfigMap ParseBTNodeRegistrationConfig(const std::string& path)
{
    return ParseBTNodeRegistrationConfig({path});
}

BTNodeRegistrationConfigMap ParseBTNodeRegistrationConfig(const std::vector<std::string>& paths)
{
    BTNodeRegistrationConfigMap map;
    for (const auto& path : paths) {
        auto new_map = YAML::LoadFile(path).as<BTNodeRegistrationConfigMap>();

        // Verify that the current file doesn't redefine a node's configuration
        for (const auto& it : map) {
            if (new_map.find(it.first) != new_map.end()) {
                throw std::runtime_error("Configuration file " + path + " introduces parameters for node '" + it.first +
                                         "' which has previously happened in another file");
            }
        }
        map.insert(new_map.begin(), new_map.end());
    }
    return map;
}

std::vector<BTNodeResource> FetchBTNodeResources(const std::string& package_name)
{
    std::string content;
    std::string base_path;
    std::vector<BTNodeResource> resources;
    if (ament_index_cpp::get_resource(_AUTO_APMS_BT_NODE_PLUGINS__RESOURCE_TYPE_NAME,
                                      package_name,
                                      content,
                                      &base_path)) {
        std::vector<std::string> lines = rcpputils::split(content, '\n', true);
        for (const auto& line : lines) {
            std::vector<std::string> parts = rcpputils::split(line, '|');
            if (parts.size() != 2) { throw std::runtime_error("Invalid resource entry"); }

            const std::string& class_name = parts[0];
            std::string library_path = base_path + "/" + parts[1];
            resources.push_back({class_name, library_path});
        }
    }
    return resources;
}

BTNodeRegistrationManifestMap CreateBTNodeRegistrationManifest(
    const BTNodeRegistrationConfigMap& registration_config_map)
{
    BTNodeRegistrationManifestMap manifest;

    // Collect resources
    std::map<std::string, std::vector<BTNodeResource>> resources;
    for (const auto& it : ament_index_cpp::get_resources(_AUTO_APMS_BT_NODE_PLUGINS__RESOURCE_TYPE_NAME)) {
        resources[it.first] = FetchBTNodeResources(it.first);
    }

    // Assemble paths of corresponding shared libraries for class_loader
    for (const auto& config : registration_config_map) {
        const auto& name = config.first;
        const auto& reg_config = config.second;
        manifest[name].registration_config = reg_config;

        // If a library path has been provided we will use this one
        if (reg_config.library.has_value() && !reg_config.package.has_value()) {
            // package_name is left empty which indicates that the library was not resolved by parsing resources
            manifest[name].package_name = "";
            manifest[name].library_path = reg_config.library.value();
            continue;
        };

        // Search for possible shared libraries to load the node from
        std::map<std::string, std::string> matching_libs;
        for (const auto& it : resources) {
            const auto& package = it.first;
            for (const auto& resource : it.second) {
                if (resource.class_name == reg_config.class_name) {
                    if (matching_libs.find(package) != matching_libs.end()) {
                        // There shouldn't be multiple matching resources in a package since this is already checked by
                        // CMake during compile time
                        throw std::runtime_error("There are multiple resource entries of " + resource.class_name);
                    }
                    matching_libs[package] = resource.library_path;
                }
            }
        }

        if (matching_libs.empty()) {
            throw std::runtime_error("The parameters for node '" + name + "' suggest to load class '" +
                                     reg_config.class_name +
                                     "', but no such resource has been registered by any package");
        }

        // Find the correct resource, verify and complete load information
        if (reg_config.package.has_value()) {
            // Verify the class name can be found
            if (matching_libs.find(reg_config.package.value()) == matching_libs.end()) {
                throw std::runtime_error("The parameters for node '" + name + "' suggest to load class '" +
                                         reg_config.class_name + "' from package '" + reg_config.package.value() +
                                         "', but no such resource has been registered by that package");
            }

            // Verify that the specified library is correct (Validation step for e.g. automatically generated configs)
            if (reg_config.library.has_value() &&
                matching_libs[reg_config.package.value()] != reg_config.library.value()) {
                throw std::runtime_error("The specified library path for node '" + name +
                                         "' does not comply with the corresponding resource of package '" +
                                         reg_config.package.value() + "'");
            }

            manifest[name].package_name = reg_config.package.value();
            manifest[name].library_path = matching_libs[reg_config.package.value()];
        }
        else {
            if (matching_libs.size() > 1) {
                // If there are multiple matching resources, a specific package name must be given
                throw std::runtime_error("There are multiple matching resources for class '" + reg_config.class_name +
                                         "', but the optional parameter '" + BT_NODE_YAML_PARAM_PACKAGE +
                                         "' was not specified for node '" + name + "'");
            }
            else {
                manifest[name].package_name = matching_libs.begin()->first;
                manifest[name].library_path = matching_libs.begin()->second;
            }
        }
    }
    return manifest;
}

}  // namespace resource

bool RegisterBTNodePlugins(rclcpp::Node::SharedPtr node_ptr,
                           BT::BehaviorTreeFactory& factory,
                           const std::string& registration_config_path)
{
    return RegisterBTNodePlugins(node_ptr, factory, {registration_config_path});
}

bool RegisterBTNodePlugins(rclcpp::Node::SharedPtr node_ptr,
                           BT::BehaviorTreeFactory& factory,
                           const std::vector<std::string>& registration_config_paths)
{
    resource::BTNodeRegistrationConfigMap registration_config;
    try {
        registration_config = resource::ParseBTNodeRegistrationConfig(registration_config_paths);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_ptr->get_logger(),
                     "RegisterBTNodePlugins: Collecting load information from configuration failed: %s",
                     e.what());
        return false;
    }

    resource::BTNodeRegistrationManifestMap manifest;
    try {
        manifest = resource::CreateBTNodeRegistrationManifest(registration_config);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_ptr->get_logger(),
                     "RegisterBTNodePlugins: Creating registration manifest failed: %s",
                     e.what());
        return false;
    }

    return RegisterBTNodePlugins(node_ptr, factory, manifest);
}

bool RegisterBTNodePlugins(rclcpp::Node::SharedPtr node_ptr,
                           BT::BehaviorTreeFactory& factory,
                           const resource::BTNodeRegistrationManifestMap& manifest_map)
{
    std::map<std::string, std::unique_ptr<class_loader::ClassLoader>> loaders;
    for (const auto& it : manifest_map) {
        const std::string& registration_name = it.first;
        const resource::BTNodeRegistrationManifest& manifest = it.second;
        if (loaders.find(manifest.library_path) == loaders.end()) {
            try {
                loaders[manifest.library_path] = std::make_unique<class_loader::ClassLoader>(manifest.library_path);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_ptr->get_logger(),
                             "RegisterBTNodePlugins: Failed to load library %s - %s",
                             manifest.library_path.c_str(),
                             e.what());
                return false;
            } catch (...) {
                RCLCPP_ERROR(node_ptr->get_logger(),
                             "RegisterBTNodePlugins: Failed to load library %s",
                             manifest.library_path.c_str());
                return false;
            }
        }
        class_loader::ClassLoader* loader = loaders[manifest.library_path].get();

        // Look if the class we search for is actually present in the library.
        const std::string factory_classname =
            "auto_apms::BTNodeRegistrarTemplate<" + manifest.registration_config.class_name + ">";
        if (!loader->isClassAvailable<BTNodeRegistrar>(factory_classname)) {
            RCLCPP_ERROR(node_ptr->get_logger(),
                         "RegisterBTNodePlugins: Class '%s' is not available in %s. You most likely misspelled the "
                         "class names of the corresponding behavior tree node in CMake when registering using "
                         "auto_apms_register_plugins()",
                         factory_classname.c_str(),
                         loader->getLibraryPath().c_str());
            return false;
        }

        auto plugin_instance = loader->createInstance<BTNodeRegistrar>(factory_classname);
        RCLCPP_DEBUG(node_ptr->get_logger(),
                     "RegisterBTNodePlugins: Register behavior tree node plugin '%s (%s)' from library %s",
                     registration_name.c_str(),
                     manifest.registration_config.class_name.c_str(),
                     manifest.library_path.c_str());

        try {
            if (plugin_instance->RequiresROSNodeParams()) {
                auto params = manifest.registration_config.CreateROSNodeParams(node_ptr);
                plugin_instance->RegisterForBehaviorTreeFactory(factory, registration_name, &params);
            }
            else {
                if (manifest.registration_config.IsROSSpecific()) {
                    RCLCPP_WARN(node_ptr->get_logger(),
                                "RegisterBTNodePlugins: ROS specific parameters were given for node '%s (%s)', but it "
                                "doesn't require any",
                                registration_name.c_str(),
                                manifest.registration_config.class_name.c_str());
                }
                plugin_instance->RegisterForBehaviorTreeFactory(factory, registration_name);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_ptr->get_logger(),
                         "RegisterBTNodePlugins: Failed to register node '%s (%s)': %s",
                         registration_name.c_str(),
                         manifest.registration_config.class_name.c_str(),
                         e.what());
            return false;
        }
    }
    return true;
}

}  // namespace auto_apms