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

#include "auto_apms/behavior_tree/behavior_tree.hpp"

#include <tinyxml2.h>

#include "auto_apms/behavior_tree/node_plugin_loader.hpp"

namespace auto_apms {

const std::string BehaviorTree::MAIN_TREE_ATTRIBUTE_NAME = "main_tree_to_execute";

struct BehaviorTree::Impl
{
    NodePluginManifest node_plugin_manifest;
    tinyxml2::XMLDocument doc;
};

BehaviorTree::BehaviorTree(const std::string& file_path) : pimpl_{std::make_unique<Impl>()}
{
    pimpl_->doc.LoadFile(file_path.c_str());
}

BehaviorTree::BehaviorTree(const Resource& resource) : BehaviorTree{resource.tree_path}
{
    pimpl_->node_plugin_manifest = NodePluginManifest::FromResource(resource);
}

BehaviorTree::~BehaviorTree() {}

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
                              const Resource& resource,
                              const std::string& main_id,
                              BT::BehaviorTreeFactory& factory,
                              BT::Blackboard::Ptr parent_blackboard_ptr)
{
    // Load behavior tree node plugins
    BTNodePluginLoader::Load(node_ptr, NodePluginManifest::FromResource(resource), factory);

    // Load tree from file
    tinyxml2::XMLDocument doc;
    doc.LoadFile(resource.tree_path.c_str());
    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);
    return Create(printer.CStr(), main_id, factory, parent_blackboard_ptr);
}

BT::Tree BehaviorTree::Create(rclcpp::Node::SharedPtr node_ptr,
                              const Resource& resource,
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
    BTNodePluginLoader::Load(node_ptr, pimpl_->node_plugin_manifest, factory);

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
    if (const auto main_tree_id = pimpl_->doc.RootElement()->Attribute(MAIN_TREE_ATTRIBUTE_NAME.c_str()))
        return main_tree_id;
    return "";
}

BehaviorTree& BehaviorTree::SetMainID(const std::string& main_tree_id)
{
    pimpl_->doc.RootElement()->SetAttribute(MAIN_TREE_ATTRIBUTE_NAME.c_str(), main_tree_id.c_str());
    return *this;
}

std::string BehaviorTree::WriteToString() const
{
    tinyxml2::XMLPrinter printer;
    pimpl_->doc.Print(&printer);
    return printer.CStr();
}

}  // namespace auto_apms