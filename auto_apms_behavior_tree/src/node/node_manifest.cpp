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

#include "auto_apms_behavior_tree/node/node_manifest.hpp"

#include <fstream>
#include <algorithm>

#include "auto_apms_behavior_tree/exceptions.hpp"
#include "yaml-cpp/yaml.h"

/// @cond
namespace YAML
{
template <>
struct convert<auto_apms_behavior_tree::NodeManifest::ParamMap>
{
  using Manifest = auto_apms_behavior_tree::NodeManifest;
  static Node encode(const Manifest::ParamMap& rhs)
  {
    Node node;
    for (const auto& [name, params] : rhs)
    {
      Node params_node;
      params_node[Manifest::PARAM_NAME_CLASS] = params.class_name;
      params_node[Manifest::PARAM_NAME_PACKAGE] = params.package;
      params_node[Manifest::PARAM_NAME_LIBRARY] = params.library;
      params_node[Manifest::PARAM_NAME_PORT] = params.port;
      params_node[Manifest::PARAM_NAME_WAIT_TIMEOUT] = params.wait_timeout.count();
      params_node[Manifest::PARAM_NAME_REQUEST_TIMEOUT] = params.request_timeout.count();
      node[name] = params_node;
    }
    return node;
  }
  static bool decode(const Node& node, Manifest::ParamMap& lhs)
  {
    if (!node.IsMap())
      throw std::runtime_error("Root YAML::Node must be map but is type " + std::to_string(node.Type()) +
                               " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");

    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
    {
      const auto& name = it->first.as<std::string>();
      const auto& params_node = it->second;
      if (!params_node.IsMap())
        throw std::runtime_error("Params YAML::Node must be map but is type " + std::to_string(params_node.Type()) +
                                 " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");

      Manifest::Params params;
      for (YAML::const_iterator p = params_node.begin(); p != params_node.end(); ++p)
      {
        const auto param_key = p->first.as<std::string>();
        const auto& val = p->second;
        if (!val.IsScalar())
          throw std::runtime_error("Value for key '" + param_key + "' must be scalar but is type " +
                                   std::to_string(val.Type()) +
                                   " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");

        if (param_key == Manifest::PARAM_NAME_CLASS)
        {
          params.class_name = val.as<std::string>();
          continue;
        }
        if (param_key == Manifest::PARAM_NAME_PORT)
        {
          params.port = val.as<std::string>();
          continue;
        }
        if (param_key == Manifest::PARAM_NAME_PACKAGE)
        {
          params.package = val.as<std::string>();
          continue;
        }
        if (param_key == Manifest::PARAM_NAME_LIBRARY)
        {
          params.library = val.as<std::string>();
          continue;
        }
        if (param_key == Manifest::PARAM_NAME_WAIT_TIMEOUT)
        {
          params.wait_timeout = std::chrono::duration<double>(val.as<double>());
          continue;
        }
        if (param_key == Manifest::PARAM_NAME_REQUEST_TIMEOUT)
        {
          params.request_timeout = std::chrono::duration<double>(val.as<double>());
          continue;
        }
        // Unkown parameter
        throw std::runtime_error("Unkown parameter '" + param_key + "'.");
      }
      lhs[name] = params;
    }
    return true;
  }
};
}  // namespace YAML
/// @endcond

namespace auto_apms_behavior_tree
{

// clang-format off
const std::string NodeManifest::PARAM_NAME_CLASS = _AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_CLASS;
const std::string NodeManifest::PARAM_NAME_PACKAGE = _AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_PACKAGE;
const std::string NodeManifest::PARAM_NAME_LIBRARY = _AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_LIBRARY;
const std::string NodeManifest::PARAM_NAME_PORT = _AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_PORT;
const std::string NodeManifest::PARAM_NAME_REQUEST_TIMEOUT = _AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_REQUEST_TIMEOUT;
const std::string NodeManifest::PARAM_NAME_WAIT_TIMEOUT = _AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_WAIT_TIMEOUT;
// clang-format on

NodeManifest::NodeManifest(const ParamMap& param_map) : param_map_{ param_map }
{
}

NodeManifest NodeManifest::fromFiles(const std::vector<std::string>& file_paths)
{
  NodeManifest manifest;
  if (file_paths.empty())
    return manifest;
  manifest = fromFile(file_paths[0]);
  for (size_t i = 1; i < file_paths.size(); ++i)
  {
    manifest.merge(fromFile(file_paths[i]));
  }
  return manifest;
}

NodeManifest NodeManifest::fromFile(const std::string& file_path)
{
  return YAML::LoadFile(file_path).as<ParamMap>();
}

NodeManifest NodeManifest::fromString(const std::string& manifest_str)
{
  const bool empty =
      std::all_of(manifest_str.begin(), manifest_str.end(), [](unsigned char c) { return std::isspace(c); });
  return empty ? ParamMap() : YAML::Load(manifest_str).as<ParamMap>();
}

bool NodeManifest::contains(const std::string& node_name) const
{
  return param_map_.find(node_name) != param_map_.end();
}

NodeManifest::Params& NodeManifest::operator[](const std::string& node_name)
{
  if (contains(node_name))
    return param_map_[node_name];
  throw std::out_of_range{ "Node '" + node_name + "' doesn't exist in manifest (Size: " +
                           std::to_string(param_map_.size()) + "). Use the Add() method to add an entry." };
}

const NodeManifest::Params& NodeManifest::operator[](const std::string& node_name) const
{
  if (contains(node_name))
    return param_map_.at(node_name);
  throw std::out_of_range{ "Node '" + node_name +
                           "' doesn't exist in manifest (Size: " + std::to_string(param_map_.size()) + ")." };
}

NodeManifest& NodeManifest::add(const std::string& node_name, const Params& p)
{
  if (contains(node_name))
  {
    throw std::logic_error{ "Node '" + node_name +
                            "' already exists in manifest (Size: " + std::to_string(param_map_.size()) + ")." };
  }
  param_map_[node_name] = p;
  return *this;
}

NodeManifest& NodeManifest::remove(const std::string& node_name)
{
  if (!contains(node_name))
  {
    throw std::logic_error{ "Node '" + node_name +
                            "' doesn't exist in manifest, so the corresponding entry cannot be removed." };
  }
  param_map_.erase(node_name);
  return *this;
}

NodeManifest& NodeManifest::merge(const NodeManifest& m)
{
  for (const auto& [node_name, params] : m.getInternalMap())
    add(node_name, params);
  return *this;
}

NodeManifest& NodeManifest::autoComplete(NodePluginClassLoader& class_loader)
{
  for (const auto& [node_name, params] : param_map_)
  {
    const std::string node_package_name = class_loader.getClassPackage(params.class_name);
    if (node_package_name.empty())
    {
      throw auto_apms_util::exceptions::ResourceNotFoundError("Cannot find class '" + params.class_name +
                                                              "' required by node '" + node_name +
                                                              "', because no such resource is registered with this "
                                                              "plugin loader instance.");
    }
    const std::string node_lib_path = class_loader.getClassLibraryPath(params.class_name);
    if (!params.package.empty())
    {
      // Verify the plugin can be found in the package
      if (params.package == node_package_name)
      {
        throw auto_apms_util::exceptions::ResourceNotFoundError(
            "Cannot find class '" + params.class_name + "' required by node '" + node_name + "' in package '" +
            params.package + "'. Internally, the resource is registered by package '" + node_package_name +
            "' instead. This can occur if multiple packages register a node plugin with the same class name. "
            "To resolve this issue, introduce a unique package namespace to the respective class names.");
      }
    }
    param_map_[node_name].package = node_package_name;
    param_map_[node_name].library = node_lib_path;  // Any entry in library will be overwritten
  }
  return *this;
}

void NodeManifest::toFile(const std::string& file_path) const
{
  YAML::Node root;
  root = param_map_;
  std::ofstream out_stream{ file_path };
  if (out_stream.is_open())
  {
    out_stream << root;
    out_stream.close();
  }
  else
  {
    throw std::runtime_error("Error opening node plugin manifest output file '" + file_path + "'.");
  }
}

std::string NodeManifest::toString() const
{
  YAML::Node root;
  root = param_map_;
  YAML::Emitter out;
  out << root;
  return out.c_str();
}

const NodeManifest::ParamMap& NodeManifest::getInternalMap() const
{
  return param_map_;
}

}  // namespace auto_apms_behavior_tree
