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

#include "auto_apms/behavior_tree/node_plugin_manifest.hpp"

#include <fstream>

#include "ament_index_cpp/get_resources.hpp"
#include "auto_apms/behavior_tree/resources.hpp"
#include "auto_apms/exceptions.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "yaml-cpp/yaml.h"

/// @internal
namespace YAML {
template <>
struct convert<auto_apms::detail::BTNodePluginManifest::ParamMap>
{
    using Manifest = auto_apms::detail::BTNodePluginManifest;
    static Node encode(const Manifest::ParamMap& rhs)
    {
        Node node;
        for (const auto& [name, params] : rhs) {
            Node params_node;
            params_node[Manifest::PARAM_NAME_CLASS] = params.class_name;
            params_node[Manifest::PARAM_NAME_PACKAGE] = params.package;
            params_node[Manifest::PARAM_NAME_LIBRARY] = params.library;
            params_node[Manifest::PARAM_NAME_PORT] = params.port;
            params_node[Manifest::PARAM_NAME_WAIT_TIMEOUT] = params.wait_timeout;
            params_node[Manifest::PARAM_NAME_REQUEST_TIMEOUT] = params.request_timeout;
            node[name] = params_node;
        }
        return node;
    }
    static bool decode(const Node& node, Manifest::ParamMap& lhs)
    {
        if (!node.IsMap()) throw std::runtime_error("Root YAML::Node must be map.");
        Manifest::ParamMap map;
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
            const auto& name = it->first.as<std::string>();
            const auto& params_node = it->second;
            if (!params_node.IsMap()) throw std::runtime_error("Params YAML::Node must be map.");

            Manifest::Params params;
            for (YAML::const_iterator p = params_node.begin(); p != params_node.end(); ++p) {
                const auto param_key = p->first.as<std::string>();
                const auto& val = p->second;
                if (!val.IsScalar()) throw std::runtime_error("Value for key '" + param_key + "' is not scalar.");

                if (param_key == Manifest::PARAM_NAME_CLASS) {
                    params.class_name = val.as<std::string>();
                    continue;
                }
                if (param_key == Manifest::PARAM_NAME_PORT) {
                    params.port = val.as<std::string>();
                    continue;
                }
                if (param_key == Manifest::PARAM_NAME_PACKAGE) {
                    params.package = val.as<std::string>();
                    continue;
                }
                if (param_key == Manifest::PARAM_NAME_LIBRARY) {
                    params.library = val.as<std::string>();
                    continue;
                }
                if (param_key == Manifest::PARAM_NAME_WAIT_TIMEOUT) {
                    params.wait_timeout = val.as<double>();
                    continue;
                }
                if (param_key == Manifest::PARAM_NAME_REQUEST_TIMEOUT) {
                    params.request_timeout = val.as<double>();
                    continue;
                }
                // Unkown parameter
                throw std::runtime_error("Unkown parameter '" + param_key + "'.");
            }
            map[name] = params;
        }
        lhs = map;
        return true;
    }
};
}  // namespace YAML
/// @endinternal

namespace auto_apms::detail {

const std::string BTNodePluginManifest::PARAM_NAME_NAMES = _AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_MANIFEST_PARAM_NAMES;
const std::string BTNodePluginManifest::PARAM_NAME_CLASS = _AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_MANIFEST_PARAM_CLASS;
const std::string BTNodePluginManifest::PARAM_NAME_PACKAGE =
    _AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_MANIFEST_PARAM_PACKAGE;
const std::string BTNodePluginManifest::PARAM_NAME_LIBRARY =
    _AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_MANIFEST_PARAM_LIBRARY;
const std::string BTNodePluginManifest::PARAM_NAME_PORT = _AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_MANIFEST_PARAM_PORT;
const std::string BTNodePluginManifest::PARAM_NAME_REQUEST_TIMEOUT =
    _AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_MANIFEST_PARAM_REQUEST_TIMEOUT;
const std::string BTNodePluginManifest::PARAM_NAME_WAIT_TIMEOUT =
    _AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_MANIFEST_PARAM_WAIT_TIMEOUT;

BTNodePluginManifest::BTNodePluginManifest(const ParamMap& param_map) : param_map_{param_map} {}

BTNodePluginManifest BTNodePluginManifest::FromResource(const BehaviorTreeResource& resource)
{
    return FromFile(resource.node_manifest_path);
}

BTNodePluginManifest BTNodePluginManifest::FromFiles(const std::vector<std::string>& file_paths)
{
    BTNodePluginManifest manifest;
    if (file_paths.empty()) return manifest;
    manifest = FromFile(file_paths[0]);
    for (size_t i = 1; i < file_paths.size(); ++i) { manifest.Merge(FromFile(file_paths[i])); }
    return manifest;
}

BTNodePluginManifest BTNodePluginManifest::FromFile(const std::string& file_path)
{
    return {YAML::LoadFile(file_path).as<ParamMap>()};
}

BTNodePluginManifest BTNodePluginManifest::Parse(const std::string& manifest_str)
{
    return {YAML::Load(manifest_str).as<ParamMap>()};
}

bool BTNodePluginManifest::Contains(const std::string& node_name) const
{
    return param_map_.find(node_name) != param_map_.end();
}

BTNodePluginManifest::Params& BTNodePluginManifest::operator[](const std::string& node_name)
{
    if (Contains(node_name)) return param_map_[node_name];
    throw std::out_of_range{"Node '" + node_name + "' doesn't exist in manifest (Size: " +
                            std::to_string(param_map_.size()) + "). Use the Add() method to add an entry."};
}

const BTNodePluginManifest::Params& BTNodePluginManifest::operator[](const std::string& node_name) const
{
    if (Contains(node_name)) return param_map_.at(node_name);
    throw std::out_of_range{"Node '" + node_name +
                            "' doesn't exist in manifest (Size: " + std::to_string(param_map_.size()) + ")."};
}

BTNodePluginManifest& BTNodePluginManifest::Add(const std::string& node_name, const Params& p)
{
    if (Contains(node_name)) {
        throw std::logic_error{"Node '" + node_name +
                               "' already exists in manifest (Size: " + std::to_string(param_map_.size()) + ")."};
    }
    param_map_[node_name] = p;
    return *this;
}

BTNodePluginManifest& BTNodePluginManifest::Remove(const std::string& node_name)
{
    if (!Contains(node_name)) {
        throw std::logic_error{"Node '" + node_name +
                               "' doesn't exist in manifest, so the corresponding entry cannot be removed."};
    }
    param_map_.erase(node_name);
    return *this;
}

BTNodePluginManifest& BTNodePluginManifest::Merge(const BTNodePluginManifest& m)
{
    for (const auto& [node_name, params] : m.map()) Add(node_name, params);
    return *this;
}

BTNodePluginManifest& BTNodePluginManifest::LocateAndVerifyLibraries(const std::set<std::string> ignore_packages)
{
    for (auto& [node_name, params] : param_map_) {
        // If a library path has been provided and no package name was specified we assume you know what you're doing
        if (!params.library.empty() && params.package.empty()) {
            // Note that params.package is left empty
            continue;
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
                        throw exceptions::ResourceNotFoundError("There are multiple resource entries of " +
                                                                resource.class_name + ".");
                    }
                    matching_libs[package_name] = resource.library_path;
                }
            }
        }

        if (matching_libs.empty()) {
            throw exceptions::ResourceNotFoundError("Cannot find class '" + params.class_name + "' required by node '" +
                                                    node_name + "', because no such resource exists in any package.");
        }

        if (!params.package.empty()) {
            // Verify the plugin can be found in the package
            if (matching_libs.find(params.package) == matching_libs.end()) {
                throw exceptions::ResourceNotFoundError("Cannot find class '" + params.class_name +
                                                        "' required by node '" + node_name + "' in package '" +
                                                        params.package + "'.");
            }

            // As an extra step, verify that the library is correct if specified
            if (!params.library.empty() && matching_libs[params.package] != params.library) {
                throw exceptions::BTNodePluginManifestError(
                    "For class '" + params.class_name + "' required by node '" + node_name +
                    "' the specified library path '" + params.library +
                    "' doesn't match with the one registered in the given package '" + params.package + "' (" +
                    matching_libs[params.package] + ").");
            }
        }
        else {
            // Proceeding here we can assume that library AND package are not specified AND there is at least one
            // associated library path (matching_lib.empty() == false)
            if (matching_libs.size() > 1) {
                // If there are multiple matching resources, a specific package name must be given
                throw exceptions::BTNodePluginManifestError(
                    "Multiple packages register resources for class '" + params.class_name + "' required by node '" +
                    node_name + "', but the optional parameter '" + PARAM_NAME_PACKAGE + "' is not specified.");
            }
            else {
                params.package = matching_libs.begin()->first;
                params.library = matching_libs.begin()->second;
            }
        }
    }
    return *this;
}

void BTNodePluginManifest::ToFile(const std::string& file_path) const
{
    YAML::Node root;
    root = param_map_;
    std::ofstream out_stream{file_path};
    if (out_stream.is_open()) {
        out_stream << root;
        out_stream.close();
    }
    else {
        throw std::runtime_error("Error opening node plugin manifest output file '" + file_path + "'.");
    }
}

std::string BTNodePluginManifest::ToString() const
{
    YAML::Node root;
    root = param_map_;
    YAML::Emitter out;
    out << root;
    return out.c_str();
}

rcl_interfaces::msg::SetParametersResult BTNodePluginManifest::ToROSParameters(rclcpp::Node::SharedPtr node_ptr,
                                                                               const std::string& prefix) const
{
    std::string dot_prefix = prefix;
    if (!dot_prefix.empty() && dot_prefix.back() != '.') dot_prefix += ".";

    std::vector<rclcpp::Parameter> param_vec;
    for (const auto& [node_name, params] : param_map_) {
        auto full_param_name = [&](const std::string& s) { return fmt::format("{}{}.{}", dot_prefix, node_name, s); };
        param_vec.push_back({dot_prefix + PARAM_NAME_NAMES, node_name});
        param_vec.push_back({full_param_name(PARAM_NAME_CLASS), params.class_name});
        param_vec.push_back({full_param_name(PARAM_NAME_LIBRARY), params.library});
        param_vec.push_back({full_param_name(PARAM_NAME_PACKAGE), params.package});
        param_vec.push_back({full_param_name(PARAM_NAME_PORT), params.port});
        param_vec.push_back({full_param_name(PARAM_NAME_WAIT_TIMEOUT), params.wait_timeout});
        param_vec.push_back({full_param_name(PARAM_NAME_REQUEST_TIMEOUT), params.request_timeout});
    }
    return node_ptr->set_parameters_atomically(param_vec);
}

const BTNodePluginManifest::ParamMap& BTNodePluginManifest::map() const { return param_map_; }

}  // namespace auto_apms::detail
