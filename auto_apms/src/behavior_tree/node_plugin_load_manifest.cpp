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

#include "auto_apms/behavior_tree/node_plugin_load_manifest.hpp"

#include <fstream>

#include "ament_index_cpp/get_resources.hpp"
#include "auto_apms/behavior_tree/resources.hpp"
#include "auto_apms/exceptions.hpp"
#include "yaml-cpp/yaml.h"

namespace YAML {
template <>
struct convert<auto_apms::detail::BTNodePluginLoadManifest>
{
    using LoadManifest = auto_apms::detail::BTNodePluginLoadManifest;
    static Node encode(const LoadManifest& rhs)
    {
        Node node;
        for (const auto& it : rhs.MapView()) {
            Node param_node;
            const auto& obj = it.second;

            // Required parameters
            param_node[LoadManifest::YAML_PARAM_CLASS] = obj.class_name;

            // Optional parameters
            if (obj.port.has_value()) param_node[LoadManifest::YAML_PARAM_PORT] = obj.port.value();
            if (obj.package.has_value()) param_node[LoadManifest::YAML_PARAM_PACKAGE] = obj.package.value();
            if (obj.library.has_value()) param_node[LoadManifest::YAML_PARAM_LIBRARY] = obj.library.value();
            if (obj.request_timeout.has_value())
                param_node[LoadManifest::YAML_PARAM_REQUEST_TIMEOUT] = obj.request_timeout.value();
            if (obj.wait_timeout.has_value())
                param_node[LoadManifest::YAML_PARAM_WAIT_TIMEOUT] = obj.wait_timeout.value();

            node[it.first] = param_node;
        }
        return node;
    }
    static bool decode(const Node& node, LoadManifest& lhs)
    {
        if (!node.IsMap()) throw std::runtime_error("Root node must be map.");
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
            const auto& name = it->first.as<std::string>();
            const auto& params_node = it->second;
            if (!params_node.IsMap()) throw std::runtime_error("Params node must be map.");

            std::set<std::string> all_param_names;
            LoadManifest::Params obj;
            for (YAML::const_iterator p = params_node.begin(); p != params_node.end(); ++p) {
                const auto param_key = p->first.as<std::string>();
                const auto& val = p->second;
                all_param_names.insert(param_key);

                auto check_scalar = [&val, &param_key]() {
                    if (!val.IsScalar()) throw std::runtime_error("Key '" + param_key + "' is not scalar.");
                };
                auto get_non_empty_string = [check_scalar, &val, &param_key]() {
                    check_scalar();
                    if (const auto str = val.as<std::string>(); !str.empty()) return str;
                    throw std::runtime_error("Key '" + param_key + "' is not allowed to be an empty string.");
                };

                if (param_key == LoadManifest::YAML_PARAM_CLASS) {
                    obj.class_name = get_non_empty_string();
                    continue;
                }

                if (param_key == LoadManifest::YAML_PARAM_PORT) {
                    obj.port = get_non_empty_string();
                    continue;
                }

                if (param_key == LoadManifest::YAML_PARAM_PACKAGE) {
                    obj.package = get_non_empty_string();
                    continue;
                }

                if (param_key == LoadManifest::YAML_PARAM_LIBRARY) {
                    obj.library = get_non_empty_string();
                    continue;
                }

                if (param_key == LoadManifest::YAML_PARAM_WAIT_TIMEOUT) {
                    check_scalar();
                    obj.wait_timeout = val.as<double>();
                    continue;
                }

                if (param_key == LoadManifest::YAML_PARAM_REQUEST_TIMEOUT) {
                    check_scalar();
                    obj.request_timeout = val.as<double>();
                    continue;
                }

                // Unkown parameter
                throw std::runtime_error("Unkown parameter '" + param_key + "'.");
            }

            // Check if all required parameters are specified
            const std::set<std::string> required_names{LoadManifest::YAML_PARAM_CLASS};
            if (!std::includes(all_param_names.begin(),
                               all_param_names.end(),
                               required_names.begin(),
                               required_names.end()))
                std::runtime_error("Registration manifest for node '" + name +
                                   "' is missing at least one required parameter.");
            lhs.Add(name, obj);
        }
        return true;
    }
};
}  // namespace YAML

namespace auto_apms::detail {

bool BTNodePluginLoadManifest::Params::IsROSSpecific() const
{
    return port.has_value() || wait_timeout.has_value() || request_timeout.has_value();
}

BT::RosNodeParams BTNodePluginLoadManifest::Params::CreateROSNodeParams(rclcpp::Node::SharedPtr node_ptr) const
{
    BT::RosNodeParams params;
    params.nh = node_ptr;
    params.default_port_value = port.value_or("");
    params.wait_for_server_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(wait_timeout.value_or(WAIT_TIMEOUT_DEFAULT_SEC)));
    params.server_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(request_timeout.value_or(REQUEST_TIMEOUT_DEFAULT_SEC)));
    return params;
}

BTNodePluginLoadManifest::BTNodePluginLoadManifest(const std::vector<std::pair<std::string, Params>> values)
{
    for (const auto& [node_name, params] : values) Add(node_name, params);
}

BTNodePluginLoadManifest BTNodePluginLoadManifest::Parse(const std::string& manifest_str)
{
    return YAML::Load(manifest_str).as<BTNodePluginLoadManifest>();
}

BTNodePluginLoadManifest BTNodePluginLoadManifest::FromFiles(const std::vector<std::string>& file_paths)
{
    BTNodePluginLoadManifest manifest;
    if (file_paths.empty()) return manifest;
    manifest = FromFile(file_paths[0]);
    for (size_t i = 1; i < file_paths.size(); ++i) { manifest.Merge(FromFile(file_paths[i])); }
    return manifest;
}

BTNodePluginLoadManifest BTNodePluginLoadManifest::FromFile(const std::string& file_path)
{
    return YAML::LoadFile(file_path).as<BTNodePluginLoadManifest>();
}

bool BTNodePluginLoadManifest::Contains(const std::string& node_name) const
{
    return param_map_.find(node_name) != param_map_.end();
}

const BTNodePluginLoadManifest::ParamMap& BTNodePluginLoadManifest::MapView() const { return param_map_; }

BTNodePluginLoadManifest::Params& BTNodePluginLoadManifest::operator[](const std::string& node_name)
{
    if (Contains(node_name)) return param_map_[node_name];
    throw std::out_of_range{"Node '" + node_name + "' doesn't exist in manifest (Size: " +
                            std::to_string(param_map_.size()) + "). Use the Add() method to add an entry."};
}

const BTNodePluginLoadManifest::Params& BTNodePluginLoadManifest::operator[](const std::string& node_name) const
{
    if (Contains(node_name)) return param_map_.at(node_name);
    throw std::out_of_range{"Node '" + node_name +
                            "' doesn't exist in manifest (Size: " + std::to_string(param_map_.size()) + ")."};
}

void BTNodePluginLoadManifest::Add(const std::string& node_name, const Params& p)
{
    if (Contains(node_name)) {
        throw std::logic_error{"Node '" + node_name +
                               "' already exists in manifest (Size: " + std::to_string(param_map_.size()) + ")."};
    }
    param_map_[node_name] = p;
}

void BTNodePluginLoadManifest::Remove(const std::string& node_name)
{
    if (!Contains(node_name)) {
        throw std::logic_error{"Node '" + node_name +
                               "' doesn't exist in manifest, so the corresponding entry cannot be removed."};
    }
    param_map_.erase(node_name);
}

void BTNodePluginLoadManifest::Merge(const BTNodePluginLoadManifest& m)
{
    for (const auto& [node_name, params] : m.MapView()) Add(node_name, params);
}

BTNodePluginLoadManifest::Params BTNodePluginLoadManifest::VerifyParameters(const Params& params,
                                                                            const std::set<std::string> ignore_packages)
{
    // Initialize a new object which is subject to change during the verification
    Params verified_params{params};

    // If a library path has been provided and no package name was specified we assume you know what you're doing
    if (params.library.has_value() && !params.package.has_value()) {
        // Note that params.package is std::nullopt which indicates that the library was not resolved by parsing
        // resources, but specified manually
        return verified_params;
    };

    // Collect resources
    std::map<std::string, std::vector<detail::BTNodeResource>> resources_map;
    for (const auto& [package_name, _] :
         ament_index_cpp::get_resources(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__NODE)) {
        if (ignore_packages.find(package_name) == ignore_packages.end()) {
            resources_map[package_name] = detail::BTNodeResource::CollectFromPackage(package_name);
        }
    }

    // Search for possible shared libraries to load the node from
    std::map<std::string, std::string> matching_libs;
    for (const auto& [package_name, resources] : resources_map) {
        for (const auto& resource : resources) {
            if (resource.class_name == params.class_name) {
                if (matching_libs.find(package_name) != matching_libs.end()) {
                    // There shouldn't be multiple matching resources in a package since this is already checked by
                    // CMake during configuration time
                    throw std::logic_error("There are multiple resource entries of " + resource.class_name + ".");
                }
                matching_libs[package_name] = resource.library_path;
            }
        }
    }

    if (matching_libs.empty()) {
        throw exceptions::ResourceNotFoundError("Cannot load BT node plugin class '" + params.class_name +
                                                "', because no such resource could be found in any package.");
    }

    if (params.package.has_value()) {
        // Verify the plugin can be found in the package
        if (matching_libs.find(params.package.value()) == matching_libs.end()) {
            throw exceptions::ResourceNotFoundError("Cannot load BT node plugin class '" + params.class_name +
                                                    "' from package '" + params.package.value() +
                                                    "', because no such resource could be found in that package.");
        }

        // As an extra step, verify that the library is correct if specified
        if (params.library.has_value() && matching_libs[params.package.value()] != params.library.value()) {
            throw exceptions::BTNodePluginManifestLogicError(
                "For class '" + params.class_name + "' the specified library path '" + params.library.value() +
                "' does not comply with the one registered in package '" + params.package.value() + "' (" +
                matching_libs[params.package.value()] + ").");
        }
    }
    else {
        // Proceeding here we can assume that library AND package are not specified AND there is at least one associated
        // library path (matching_lib.empty() == false)
        if (matching_libs.size() > 1) {
            // If there are multiple matching resources, a specific package name must be given
            throw exceptions::BTNodePluginManifestLogicError(
                "Multiple packages register resources for BT node plugin class '" + params.class_name +
                "', but the optional parameter '" + YAML_PARAM_PACKAGE + "' was not specified.");
        }
        else {
            verified_params.package = matching_libs.begin()->first;
            verified_params.library = matching_libs.begin()->second;
        }
    }
    return verified_params;
}

BTNodePluginLoadManifest BTNodePluginLoadManifest::Verify() const
{
    BTNodePluginLoadManifest verified_manifest;
    for (const auto& [node_name, params] : this->MapView()) {
        verified_manifest.Add(node_name, VerifyParameters(params));
    }
    return verified_manifest;
}

void BTNodePluginLoadManifest::ToFile(const std::string& file_path) const
{
    YAML::Node root;
    root = *this;
    std::ofstream out_stream{file_path};
    if (out_stream.is_open()) {
        out_stream << root;
        out_stream.close();
    }
    else {
        throw std::runtime_error("Error opening node plugin manifest output file '" + file_path + "'.");
    }
}

std::string BTNodePluginLoadManifest::ToString() const
{
    YAML::Node root;
    root = *this;
    YAML::Emitter out;
    out << root;
    return out.c_str();
}

}  // namespace auto_apms::detail
