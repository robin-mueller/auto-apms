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

#include <filesystem>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace px4_behavior {

/// @brief Struct for behavior tree node plugin resources
struct BTNodePluginResource
{
    std::string classname;
    std::string library_path;
};

/// @brief Struct for behavior tree resources
struct BehaviorTreeResource
{
    std::string tree_file_name;
    std::string tree_path;
    std::set<std::string> plugin_config_paths;
    std::set<std::string> tree_ids;
};

/**
 * @brief Read a trees file.
 *
 * Read the data of the file @p tree_path and return the raw string.
 *
 * @param tree_path Path to the file containing the behavior tree's definition
 * @return XML string of the behavior tree file.
 * @throw std::runtime_error if failed to read file.
 */
std::string ReadBehaviorTreeFile(const std::filesystem::path& tree_path);

std::vector<BTNodePluginResource> FetchBTNodePluginResources(const std::string& package_name);

std::vector<BehaviorTreeResource> FetchBehaviorTreeResources(const std::string& package_name);

std::optional<BehaviorTreeResource> FetchBehaviorTreeResource(
    std::optional<const std::string> tree_file_name = std::nullopt,
    std::optional<const std::string> tree_id = std::nullopt,
    std::optional<const std::string> package_name = std::nullopt);

}  // namespace px4_behavior