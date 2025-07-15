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

#include "auto_apms_behavior_tree_core/behavior.hpp"
#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_util/resource.hpp"
#include "auto_apms_util/string.hpp"

namespace auto_apms_behavior_tree::core
{

NodeManifest::NodeManifest(const Map & map) : map_{map} {}

NodeManifest NodeManifest::fromFiles(const std::vector<std::string> & paths)
{
  NodeManifest manifest;
  for (const auto & path : paths) {
    try {
      manifest.merge(fromFile(path));
    } catch (const std::exception & e) {
      throw exceptions::NodeManifestError("Error creating node manifest from multiple files: " + std::string(e.what()));
    }
  }
  return manifest;
}

NodeManifest NodeManifest::fromResourceIdentity(const std::string & identity)
{
  const auto tokens = auto_apms_util::splitString(identity, RESOURCE_IDENTITY_RESOURCE_SEPARATOR, false);
  std::string package_name = "";
  std::string file_stem;
  switch (tokens.size()) {
    case 1:
      file_stem = tokens[0];
      break;
    case 2:
      package_name = tokens[0];
      file_stem = tokens[1];
      break;
    default:
      throw auto_apms_util::exceptions::ResourceIdentityFormatError(
        "Node manifest resource identity string '" + identity + "' has wrong format. Must be '<package_name>" +
        RESOURCE_IDENTITY_RESOURCE_SEPARATOR + "<metadata_id>'.");
  }

  std::set<std::string> search_packages;
  if (!package_name.empty()) {
    search_packages.insert(package_name);
  } else {
    search_packages =
      auto_apms_util::getPackagesWithResourceType(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__NODE_MANIFEST);
  }

  std::vector<std::string> matching_file_paths;
  for (const auto & p : search_packages) {
    std::string content;
    std::string base_path;
    if (ament_index_cpp::get_resource(
          _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__NODE_MANIFEST, p, content, &base_path)) {
      std::vector<std::string> lines = auto_apms_util::splitString(content, "\n");
      for (const std::string & line : lines) {
        std::vector<std::string> parts = auto_apms_util::splitString(line, "|", false);
        if (parts.size() != 2) {
          throw auto_apms_util::exceptions::ResourceError(
            "Invalid node manifest resource file (Package: '" + p + "'). Invalid line: " + line + ".");
        }
        if (parts[0] == file_stem) {
          matching_file_paths.push_back(base_path + "/" + parts[1]);
        }
      }
    }
  }

  if (matching_file_paths.empty()) {
    throw auto_apms_util::exceptions::ResourceError(
      "No node manifest resource was found using identity '" + identity + "'.");
  }
  if (matching_file_paths.size() > 1) {
    throw auto_apms_util::exceptions::ResourceError(
      "There are multiple node manifest resources with metadata ID '" + file_stem + "'.");
  }
  return fromFile(matching_file_paths[0]);
}

void NodeManifest::toFile(const std::string & file_path) const
{
  std::ofstream out_stream(file_path);
  if (out_stream.is_open()) {
    out_stream << this->encode();
    out_stream.close();
  } else {
    throw exceptions::NodeManifestError("Error opening node manifest output file '" + file_path + "'.");
  }
}

bool NodeManifest::contains(const std::string & node_name) const { return map_.find(node_name) != map_.end(); }

NodeManifest::RegistrationOptions & NodeManifest::operator[](const std::string & node_name)
{
  if (contains(node_name)) return map_[node_name];
  throw std::out_of_range{
    "Node '" + node_name + "' doesn't exist in node manifest (Size: " + std::to_string(map_.size()) +
    "). Use the add() method to add an entry."};
}

const NodeManifest::RegistrationOptions & NodeManifest::operator[](const std::string & node_name) const
{
  if (contains(node_name)) return map_.at(node_name);
  throw std::out_of_range{
    "Node '" + node_name + "' doesn't exist in node manifest (Size: " + std::to_string(map_.size()) + ")."};
}

NodeManifest & NodeManifest::add(const std::string & node_name, const RegistrationOptions & opt)
{
  if (contains(node_name)) {
    throw exceptions::NodeManifestError{
      "Node '" + node_name + "' already exists in node manifest (Size: " + std::to_string(map_.size()) + ")."};
  }
  if (!opt.valid()) {
    throw exceptions::NodeManifestError(
      "Cannot add node '" + node_name + "' to manifest. Parameter class_name must not be empty.");
  }
  map_[node_name] = opt;
  return *this;
}

NodeManifest & NodeManifest::remove(const std::string & node_name)
{
  if (!contains(node_name)) {
    throw std::out_of_range{
      "Node '" + node_name + "' doesn't exist in node manifest, so the corresponding entry cannot be removed."};
  }
  map_.erase(node_name);
  return *this;
}

NodeManifest & NodeManifest::merge(const NodeManifest & other, bool replace)
{
  for (const auto & [node_name, params] : other.map()) {
    if (contains(node_name)) {
      if (replace) {
        map_.erase(node_name);
      } else {
        throw exceptions::NodeManifestError(
          "Cannot merge node manifests, because node '" + node_name +
          "' already exists in other and argument replace is false (Won't replace existing entries).");
      }
    }
    add(node_name, params);
  }
  return *this;
}

std::vector<std::string> NodeManifest::getNodeNames()
{
  std::vector<std::string> names;
  names.reserve(map_.size());
  for (const auto & [name, _] : map_) names.push_back(name);
  return names;
}

size_t NodeManifest::size() const { return map_.size(); }

bool NodeManifest::empty() const { return map_.empty(); }

const NodeManifest::Map & NodeManifest::map() const { return map_; }

}  // namespace auto_apms_behavior_tree::core
