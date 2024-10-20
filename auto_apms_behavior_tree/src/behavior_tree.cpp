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

#include "auto_apms_behavior_tree/behavior_tree.hpp"

#include "ament_index_cpp/get_resource.hpp"
#include "auto_apms_behavior_tree/exceptions.hpp"
#include "auto_apms_behavior_tree/node_plugin_loader.hpp"
#include "auto_apms_core/resources.hpp"
#include "rcpputils/split.hpp"

namespace auto_apms_behavior_tree {

std::vector<BTResource> BTResource::CollectFromPackage(const std::string& package_name)
{
    std::string content;
    std::string base_path;
    std::vector<BTResource> resources;
    if (ament_index_cpp::get_resource(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__TREE,
                                      package_name,
                                      content,
                                      &base_path)) {
        std::vector<std::string> lines = rcpputils::split(content, '\n', true);
        auto make_absolute_path = [base_path](const std::string& s) { return base_path + "/" + s; };
        for (const auto& line : lines) {
            std::vector<std::string> parts = rcpputils::split(line, '|', false);
            if (parts.size() != 4) {
                throw std::runtime_error("Invalid behavior tree resource file (Package: '" + package_name + "').");
                ;
            }

            BTResource r;
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

BTResource BTResource::SelectByID(const std::string& tree_id, const std::string& package_name)
{
    std::set<std::string> search_packages;
    if (!package_name.empty()) { search_packages.insert(package_name); }
    else {
        search_packages =
            auto_apms_core::GetAllPackagesWithResource(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__TREE);
    }

    std::vector<BTResource> matching_resources;
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

BTResource BTResource::SelectByFileName(const std::string& file_name, const std::string& package_name)
{
    const std::string file_stem = std::filesystem::path{file_name}.stem().string();
    std::set<std::string> search_packages;
    if (!package_name.empty()) { search_packages.insert(package_name); }
    else {
        search_packages =
            auto_apms_core::GetAllPackagesWithResource(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__TREE);
    }

    std::vector<BTResource> matching_resources;
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

BehaviorTree::BehaviorTree(const std::string& file_path, const NodePluginManifest& node_plugin_manifest)
    : node_plugin_manifest_{node_plugin_manifest}
{
    doc_.LoadFile(file_path.c_str());
}

BehaviorTree::BehaviorTree(const BTResource& resource)
    : BehaviorTree{resource.tree_path, NodePluginManifest::FromFile(resource.node_manifest_path)}
{}

BT::Tree BehaviorTree::Create(const std::string& tree_str,
                              const std::string& main_id,
                              BT::BehaviorTreeFactory& factory,
                              BT::Blackboard::Ptr parent_blackboard_ptr)
{
    // Create empty blackboard if none was provided
    if (!parent_blackboard_ptr) parent_blackboard_ptr = BT::Blackboard::create();

    factory.registerBehaviorTreeFromText(tree_str);
    return factory.createTree(main_id, parent_blackboard_ptr);
}

BT::Tree BehaviorTree::Create(rclcpp::Node::SharedPtr node_ptr,
                              const BTResource& resource,
                              const std::string& main_id,
                              BT::BehaviorTreeFactory& factory,
                              BT::Blackboard::Ptr parent_blackboard_ptr)
{
    // Load behavior tree node plugins
    BTNodePluginLoader{node_ptr}.Load(NodePluginManifest::FromFile(resource.node_manifest_path), factory);

    // Load tree from file
    tinyxml2::XMLDocument doc;
    doc.LoadFile(resource.tree_path.c_str());
    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);
    return Create(printer.CStr(), main_id, factory, parent_blackboard_ptr);
}

BT::Tree BehaviorTree::Create(rclcpp::Node::SharedPtr node_ptr,
                              const BTResource& resource,
                              const std::string& main_id,
                              BT::Blackboard::Ptr parent_blackboard_ptr)
{
    BT::BehaviorTreeFactory factory;
    return Create(node_ptr, resource, main_id, factory, parent_blackboard_ptr);
}

BT::Tree BehaviorTree::Create(rclcpp::Node::SharedPtr node_ptr,
                              BT::BehaviorTreeFactory& factory,
                              BT::Blackboard::Ptr parent_blackboard_ptr) const
{
    // Load behavior tree node plugins
    BTNodePluginLoader{node_ptr}.Load(node_plugin_manifest_, factory);

    // Create behavior tree using main tree attribute
    return Create(WriteToString(), "", factory, parent_blackboard_ptr);
}

BT::Tree BehaviorTree::Create(rclcpp::Node::SharedPtr node_ptr, BT::Blackboard::Ptr parent_blackboard_ptr) const
{
    BT::BehaviorTreeFactory factory;
    return Create(node_ptr, factory, parent_blackboard_ptr);
}

std::string BehaviorTree::GetMainID() const
{
    if (const auto main_tree_id = doc_.RootElement()->Attribute(MAIN_TREE_ATTRIBUTE_NAME.c_str())) return main_tree_id;
    return "";
}

BehaviorTree& BehaviorTree::SetMainID(const std::string& main_tree_id)
{
    doc_.RootElement()->SetAttribute(MAIN_TREE_ATTRIBUTE_NAME.c_str(), main_tree_id.c_str());
    return *this;
}

std::string BehaviorTree::WriteToString() const
{
    tinyxml2::XMLPrinter printer;
    doc_.Print(&printer);
    return printer.CStr();
}

}  // namespace auto_apms_behavior_tree