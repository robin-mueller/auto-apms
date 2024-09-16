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

#include "auto_apms/resource/tree.hpp"

#include <fstream>
#include <functional>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_resource.hpp"
#include "ament_index_cpp/get_resources.hpp"
#include "auto_apms/resource/node.hpp"
#include "rcpputils/split.hpp"

namespace auto_apms {
namespace resource {

std::vector<BehaviorTreeResource> FetchBehaviorTreeResources(const std::string& package_name)
{
    std::string content;
    std::string base_path;
    std::vector<BehaviorTreeResource> resources;
    if (ament_index_cpp::get_resource(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME,
                                      package_name,
                                      content,
                                      &base_path)) {
        std::vector<std::string> lines = rcpputils::split(content, '\n', true);
        auto make_absolute_path = [base_path](const std::string& s) { return base_path + "/" + s; };
        for (const auto& line : lines) {
            std::vector<std::string> parts = rcpputils::split(line, '|');
            if (parts.size() != 4) { throw std::runtime_error("Invalid resource entry"); }

            const std::string& filename = parts[0];
            std::string tree_path = make_absolute_path(parts[1]);
            std::vector<std::string> plugin_config_paths_vec = rcpputils::split(parts[2], ';');
            std::transform(plugin_config_paths_vec.begin(),
                           plugin_config_paths_vec.end(),
                           plugin_config_paths_vec.begin(),
                           make_absolute_path);

            std::vector<std::string> tree_ids_vec = rcpputils::split(parts[3], ';');
            resources.push_back({filename,
                                 tree_path,
                                 {plugin_config_paths_vec.begin(), plugin_config_paths_vec.end()},
                                 {tree_ids_vec.begin(), tree_ids_vec.end()}});
        }
    }
    return resources;
}

std::optional<BehaviorTreeResource> FetchBehaviorTreeResource(std::optional<const std::string> tree_file_name,
                                                              std::optional<const std::string> tree_id,
                                                              std::optional<const std::string> package_name)
{
    std::vector<std::string> search_packages;
    if (package_name.has_value()) { search_packages.push_back(package_name.value()); }
    else {
        for (const auto& r : ament_index_cpp::get_resources(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME)) {
            search_packages.push_back(r.first);
        }
    }

    std::vector<BehaviorTreeResource> matching_resources;
    std::function<bool(const BehaviorTreeResource&)> is_resource_matching;
    if (tree_file_name.has_value()) {
        is_resource_matching = [fn = tree_file_name.value()](const BehaviorTreeResource& r) {
            return r.tree_file_name == fn;
        };
    }
    else if (tree_id.has_value()) {
        is_resource_matching = [tid = tree_id.value()](const BehaviorTreeResource& r) {
            return r.tree_ids.find(tid) != r.tree_ids.end();
        };
    }
    else {
        throw std::runtime_error("Either of arguments tree_file_name or tree_id must be specified");
    }

    for (const auto& package_name : search_packages) {
        const auto resources = FetchBehaviorTreeResources(package_name);
        if (resources.empty()) { continue; }
        for (const auto& r : resources) {
            if (is_resource_matching(r)) { matching_resources.push_back(r); }
        }
    }

    if (matching_resources.empty()) { return {}; }
    if (matching_resources.size() > 1) {
        throw std::runtime_error("There are multiple matching behavior tree resources");
    }
    return matching_resources[0];
}

std::string ReadBehaviorTreeFile(const std::filesystem::path& tree_path)
{
    // Note that we have to use binary mode as we want to return a string matching the bytes of the file
    std::ifstream file(tree_path, std::ios::binary);
    if (!file.is_open()) { throw std::runtime_error("Couldn't open file '" + tree_path.string() + "'"); }

    // Read contents
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

}  // namespace resource

BT::Tree CreateBehaviorTree(rclcpp::Node::SharedPtr node_ptr,
                            std::optional<const std::string> tree_file_name,
                            std::optional<const std::string> tree_id,
                            std::optional<const std::string> package_name)
{
    BT::BehaviorTreeFactory factory;
    auto parent_blackboard = BT::Blackboard::create();
    return CreateBehaviorTree(node_ptr, factory, parent_blackboard, tree_file_name, tree_id, package_name);
}

BT::Tree CreateBehaviorTree(rclcpp::Node::SharedPtr node_ptr,
                            BT::BehaviorTreeFactory& factory,
                            std::optional<const std::string> tree_file_name,
                            std::optional<const std::string> tree_id,
                            std::optional<const std::string> package_name)
{
    auto parent_blackboard = BT::Blackboard::create();
    return CreateBehaviorTree(node_ptr, factory, parent_blackboard, tree_file_name, tree_id, package_name);
}

BT::Tree CreateBehaviorTree(rclcpp::Node::SharedPtr node_ptr,
                            const BT::Blackboard::Ptr& parent_blackboard,
                            std::optional<const std::string> tree_file_name,
                            std::optional<const std::string> tree_id,
                            std::optional<const std::string> package_name)
{
    BT::BehaviorTreeFactory factory;
    return CreateBehaviorTree(node_ptr, factory, parent_blackboard, tree_file_name, tree_id, package_name);
}

BT::Tree CreateBehaviorTree(rclcpp::Node::SharedPtr node_ptr,
                            BT::BehaviorTreeFactory& factory,
                            const BT::Blackboard::Ptr& parent_blackboard,
                            std::optional<const std::string> tree_file_name,
                            std::optional<const std::string> tree_id,
                            std::optional<const std::string> package_name)
{
    std::optional<resource::BehaviorTreeResource> bt_resource_opt;
    try {
        bt_resource_opt = resource::FetchBehaviorTreeResource(tree_file_name, tree_id, package_name);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_ptr->get_logger(),
                     "CreateBehaviorTree: Error fetching behavior tree resource for tree_file_name='%s', "
                     "tree_id='%s', package_name='%s': %s",
                     tree_file_name.value_or("").c_str(),
                     tree_id.value_or("").c_str(),
                     package_name.value_or("").c_str(),
                     e.what());
        return {};
    }
    if (!bt_resource_opt.has_value()) {
        RCLCPP_ERROR(node_ptr->get_logger(),
                     "CreateBehaviorTree: No behavior tree resource found for tree_file_name='%s', "
                     "tree_id='%s', package_name='%s'",
                     tree_file_name.value_or("").c_str(),
                     tree_id.value_or("").c_str(),
                     package_name.value_or("").c_str());
        return {};
    }
    const resource::BehaviorTreeResource& bt_resource = bt_resource_opt.value();

    // Regsiter associated node plugins
    if (!RegisterBTNodePlugins(node_ptr, factory, bt_resource.plugin_config_paths)) {
        RCLCPP_ERROR(node_ptr->get_logger(), "CreateBehaviorTree: Registering behavior tree node plugins failed");
        return {};
    }

    // Register behavior tree resource file
    try {
        factory.registerBehaviorTreeFromFile(bt_resource.tree_path);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_ptr->get_logger(),
                     "CreateBehaviorTree: Error registering behavior tree resource file %s: %s",
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
        RCLCPP_ERROR(node_ptr->get_logger(),
                     "CreateBehaviorTree: Error creating tree with ID %s from file %s: %s",
                     tid.c_str(),
                     bt_resource.tree_path.c_str(),
                     e.what());
        return {};
    }

    return tree;
}

}  // namespace auto_apms