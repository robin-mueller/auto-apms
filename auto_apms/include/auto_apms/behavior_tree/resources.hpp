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

#include <set>
#include <string>
#include <vector>

namespace auto_apms {
namespace detail {

/// @brief Struct for behavior tree node plugin resource data
struct BTNodeResource
{
    std::string class_name;    ///< Name of the class that can be loaded from BTNodeResource::library_path
    std::string library_path;  ///< Path to the library associated with this resource

    /**
     * @brief Collect all behavior tree node plugin resources registered by a certain package.
     * @param package_name Name of the package to search for resources.
     * @return Collection of all resources found in @p package_name.
     */
    static std::vector<BTNodeResource> CollectFromPackage(const std::string& package_name);
};

}  // namespace detail

/// @brief Struct for behavior tree resource data
struct BehaviorTreeResource
{
    std::string name;
    std::string tree_path;
    std::string node_manifest_path;
    std::set<std::string> tree_ids;

    /**
     * @brief Collect all behavior tree resources registered by a certain package.
     * @param package_name Name of the package to search for resources.
     * @return Collection of all resources found in @p package_name.
     */
    static std::vector<BehaviorTreeResource> CollectFromPackage(const std::string& package_name);

    static BehaviorTreeResource SelectByID(const std::string& tree_id, const std::string& package_name = "");

    static BehaviorTreeResource SelectByFileName(const std::string& file_name, const std::string& package_name = "");
};

}  // namespace auto_apms
