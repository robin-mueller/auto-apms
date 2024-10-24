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

#include "auto_apms_behavior_tree/creator.hpp"

#include "auto_apms_behavior_tree/exceptions.hpp"
#include "behaviortree_cpp/xml_parsing.h"

namespace auto_apms_behavior_tree {

BTCreator::BTCreator(const BTNodePluginLoader& node_plugin_loader) : node_plugin_loader_{node_plugin_loader}
{
    doc_.Parse("<root BTCPP_format=\"4\"></root>");
    BT::VerifyXML(WriteToString(), {});
}

BTCreator::SharedPtr BTCreator::FromResource(rclcpp::Node::SharedPtr node_ptr, const BTResource& resource)
{
    SharedPtr ptr = std::make_shared<BTCreator>();
    ptr->AddTreeFromFile(resource.tree_file_path,
                         BTNodePluginManifest::FromFile(resource.node_manifest_file_path),
                         node_ptr);
    return ptr;
}

void BTCreator::AddTreeFromString(const std::string& tree_str,
                                  const BTNodePluginManifest& node_plugin_manifest,
                                  rclcpp::Node::SharedPtr node_ptr)
{
    tinyxml2::XMLDocument new_doc;
    if (new_doc.Parse(tree_str.c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
        throw exceptions::BTCreatorError("Cannot add tree: " + std::string(new_doc.ErrorStr()));
    }
    AddTreeFromXMLDocument(new_doc, node_plugin_manifest, node_ptr);
}

void BTCreator::AddTreeFromFile(const std::string& tree_file_path,
                                const BTNodePluginManifest& node_plugin_manifest,
                                rclcpp::Node::SharedPtr node_ptr)
{
    tinyxml2::XMLDocument new_doc;
    if (new_doc.LoadFile(tree_file_path.c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
        throw exceptions::BTCreatorError("Cannot add tree: " + std::string(new_doc.ErrorStr()));
    }
    AddTreeFromXMLDocument(new_doc, node_plugin_manifest, node_ptr);
}

void BTCreator::AddTreeFromXMLDocument(const tinyxml2::XMLDocument& doc,
                                       const BTNodePluginManifest& node_plugin_manifest,
                                       rclcpp::Node::SharedPtr node_ptr)
{
    auto original_root = doc_.RootElement();
    auto new_root = doc.RootElement();
    if (!new_root) throw exceptions::BTCreatorError("Cannot merge new XMLDocument: new_root is nullptr.");

    // Iterate over all the children of the new document's root element
    for (const tinyxml2::XMLElement* child = new_root->FirstChildElement(); child != nullptr;
         child = child->NextSiblingElement()) {
        // Clone the child element to the original document
        auto copied_child = child->DeepClone(&doc_);
        // Append the copied child to the original document's root
        original_root->InsertEndChild(copied_child);
    }

    // Add node plugins to factory
    node_plugin_loader_.Load(node_plugin_manifest, node_ptr, factory_);

    // Collect the names of all nodes registered with the behavior tree factory
    std::unordered_map<std::string, BT::NodeType> registered_nodes;
    for (const auto& it : factory_.manifests()) registered_nodes.insert({it.first, it.second.type});

    // Verify the new XML document
    BT::VerifyXML(WriteToString(), registered_nodes);
}

BT::Tree BTCreator::CreateTree(const std::string& main_tree_id, BT::Blackboard::Ptr root_blackboard_ptr)
{
    factory_.registerBehaviorTreeFromText(WriteToString());
    BT::Tree tree = factory_.createTree(main_tree_id, root_blackboard_ptr);
    factory_.clearRegisteredBehaviorTrees();
    return tree;
}

BT::Tree BTCreator::CreateMainTree(BT::Blackboard::Ptr root_blackboard_ptr)
{
    return CreateTree("", root_blackboard_ptr);
}

std::string BTCreator::GetMainTreeName() const
{
    if (const auto main_tree_id = doc_.RootElement()->Attribute(MAIN_TREE_ATTRIBUTE_NAME.c_str())) return main_tree_id;
    return "";
}

void BTCreator::SetMainTreeName(const std::string& main_tree_id)
{
    if (!main_tree_id.empty()) doc_.RootElement()->SetAttribute(MAIN_TREE_ATTRIBUTE_NAME.c_str(), main_tree_id.c_str());
}

std::string BTCreator::WriteToString() const { return WriteXMLDocumentToString(doc_); }

std::string BTCreator::WriteXMLDocumentToString(const tinyxml2::XMLDocument& doc)
{
    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);
    return printer.CStr();
}

BT::BehaviorTreeFactory& BTCreator::factory() { return factory_; }

}  // namespace auto_apms_behavior_tree