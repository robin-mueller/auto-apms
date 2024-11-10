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

#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"

#include <fstream>

#include "auto_apms_behavior_tree_core/exceptions.hpp"

namespace auto_apms_behavior_tree::core
{

NodeManifest::NodeManifest(const ParamMap & param_map) : param_map_{param_map} {}

NodeManifest NodeManifest::fromFile(const std::string & file_path)
{
  return YAML::LoadFile(file_path).as<NodeManifest>();
}

NodeManifest NodeManifest::fromFiles(const std::vector<std::string> & file_paths)
{
  NodeManifest manifest;
  for (const auto & path : file_paths) manifest.merge(fromFile(path));
  return manifest;
}

void NodeManifest::toFile(const std::string & file_path) const
{
  std::ofstream out_stream{file_path};
  if (out_stream.is_open()) {
    out_stream << this->encode();
    out_stream.close();
  } else {
    throw std::runtime_error("Error opening node manifest output file '" + file_path + "'.");
  }
}

bool NodeManifest::contains(const std::string & node_name) const
{
  return param_map_.find(node_name) != param_map_.end();
}

NodeManifest::Params & NodeManifest::operator[](const std::string & node_name)
{
  if (contains(node_name)) return param_map_[node_name];
  throw std::out_of_range{
    "Node '" + node_name + "' doesn't exist in node manifest (Size: " + std::to_string(param_map_.size()) +
    "). Use the add() method to add an entry."};
}

const NodeManifest::Params & NodeManifest::operator[](const std::string & node_name) const
{
  if (contains(node_name)) return param_map_.at(node_name);
  throw std::out_of_range{
    "Node '" + node_name + "' doesn't exist in node manifest (Size: " + std::to_string(param_map_.size()) + ")."};
}

NodeManifest & NodeManifest::add(const std::string & node_name, const Params & p)
{
  if (contains(node_name)) {
    throw std::logic_error{
      "Node '" + node_name + "' already exists in node manifest (Size: " + std::to_string(param_map_.size()) + ")."};
  }

  // Validate parameters
  if (p.class_name.empty()) {
    throw std::logic_error("Cannot add node '" + node_name + "' to manifest. Parameter class_name must not be empty.");
  }

  param_map_[node_name] = p;
  return *this;
}

NodeManifest & NodeManifest::remove(const std::string & node_name)
{
  if (!contains(node_name)) {
    throw std::logic_error{
      "Node '" + node_name + "' doesn't exist in node manifest, so the corresponding entry cannot be removed."};
  }
  param_map_.erase(node_name);
  return *this;
}

NodeManifest & NodeManifest::merge(const NodeManifest & m)
{
  for (const auto & [node_name, params] : m.getInternalMap()) add(node_name, params);
  return *this;
}

const NodeManifest::ParamMap & NodeManifest::getInternalMap() const { return param_map_; }

}  // namespace auto_apms_behavior_tree::core
