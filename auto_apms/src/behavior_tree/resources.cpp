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

#include "auto_apms/behavior_tree/resources.hpp"

#include <filesystem>
#include <functional>

#include "ament_index_cpp/get_resource.hpp"
#include "ament_index_cpp/get_resources.hpp"
#include "auto_apms/exceptions.hpp"
#include "rcpputils/split.hpp"

namespace auto_apms {
namespace detail {

std::vector<BTNodeResource> BTNodeResource::CollectFromPackage(const std::string& package_name)
{
    std::string content;
    std::string base_path;
    std::vector<BTNodeResource> resources;
    if (ament_index_cpp::get_resource(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__NODE,
                                      package_name,
                                      content,
                                      &base_path)) {
        std::vector<std::string> lines = rcpputils::split(content, '\n', true);
        for (const auto& line : lines) {
            std::vector<std::string> parts = rcpputils::split(line, '|');
            if (parts.size() != 2) { throw std::runtime_error("Invalid resource entry"); }

            BTNodeResource r;
            r.class_name = parts[0];
            r.library_path = base_path + "/" + parts[1];
            resources.push_back(r);
        }
    }
    return resources;
}

}  // namespace detail

std::vector<BehaviorTreeResource> BehaviorTreeResource::CollectFromPackage(const std::string& package_name)
{
    std::string content;
    std::string base_path;
    std::vector<BehaviorTreeResource> resources;
    if (ament_index_cpp::get_resource(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__TREE,
                                      package_name,
                                      content,
                                      &base_path)) {
        std::vector<std::string> lines = rcpputils::split(content, '\n', true);
        auto make_absolute_path = [base_path](const std::string& s) { return base_path + "/" + s; };
        for (const auto& line : lines) {
            std::vector<std::string> parts = rcpputils::split(line, '|', false);
            if (parts.size() != 4) { throw std::runtime_error("Invalid resource entry"); }

            BehaviorTreeResource r;
            r.name = parts[0];
            r.tree_path = make_absolute_path(parts[1]);
            r.node_manifest_path = make_absolute_path(parts[2]);
            std::vector<std::string> tree_ids_vec = rcpputils::split(parts[3], ';');
            r.tree_ids = {tree_ids_vec.begin(), tree_ids_vec.end()};
            resources.push_back(r);
        }
    }
    return resources;
}

BehaviorTreeResource BehaviorTreeResource::SelectByID(const std::string& tree_id, const std::string& package_name)
{
    std::vector<std::string> search_packages;
    if (!package_name.empty()) { search_packages.push_back(package_name); }
    else {
        for (const auto& r : ament_index_cpp::get_resources(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__TREE)) {
            search_packages.push_back(r.first);
        }
    }

    std::vector<BehaviorTreeResource> matching_resources;
    for (const auto& package_name : search_packages) {
        for (const auto& r : CollectFromPackage(package_name)) {
            if (r.tree_ids.find(tree_id) != r.tree_ids.end()) { matching_resources.push_back(r); }
        }
    }

    if (matching_resources.empty()) {
        throw exceptions::ResourceNotFoundError{"No behavior tree with ID '" + tree_id + "' was registered."};
    }
    if (matching_resources.size() > 1) {
        throw exceptions::ResourceNotFoundError{
            "The behavior tree ID '" + tree_id +
            "' exists multiple times. Use the 'package_name' argument to narrow down the search."};
    }

    return matching_resources[0];
}

BehaviorTreeResource BehaviorTreeResource::SelectByFileName(const std::string& file_name,
                                                            const std::string& package_name)
{
    const std::string file_stem = std::filesystem::path{file_name}.stem().string();
    std::vector<std::string> search_packages;
    if (!package_name.empty()) { search_packages.push_back(package_name); }
    else {
        for (const auto& r : ament_index_cpp::get_resources(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__TREE)) {
            search_packages.push_back(r.first);
        }
    }

    std::vector<BehaviorTreeResource> matching_resources;
    for (const auto& package_name : search_packages) {
        for (const auto& r : CollectFromPackage(package_name)) {
            if (r.name == file_stem) { matching_resources.push_back(r); }
        }
    }

    if (matching_resources.empty()) {
        throw exceptions::ResourceNotFoundError{"No behavior tree file with name '" + file_stem +
                                                ".xml' was registered."};
    }
    if (matching_resources.size() > 1) {
        throw exceptions::ResourceNotFoundError{
            "Multiple behavior tree files with name '" + file_stem +
            ".xml' are registered. Use the 'package_name' argument to narrow down the search."};
    }

    return matching_resources[0];
}

}  // namespace auto_apms