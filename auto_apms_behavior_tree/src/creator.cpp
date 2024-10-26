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
#include "auto_apms_core/resources.hpp"
#include "auto_apms_core/util/split.hpp"
#include "behaviortree_cpp/xml_parsing.h"

namespace auto_apms_behavior_tree {

std::shared_ptr<BTNodePluginClassLoader> MakeBTNodePluginClassLoader(const std::set<std::string>& package_names)
{
    std::vector<std::string> xml_paths;
    for (const auto& package_name : package_names.empty() ? auto_apms_core::GetAllPackagesWithResource(
                                                                _AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__NODE)
                                                          : package_names) {
        std::string content;
        std::string base_path;
        if (ament_index_cpp::get_resource(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__NODE,
                                          package_name,
                                          content,
                                          &base_path)) {
            std::vector<std::string> paths = auto_apms_core::util::SplitString(content, "\n", false);
            if (paths.size() != 1) {
                throw std::runtime_error("Invalid behavior tree node plugin resource file (Package: '" + package_name +
                                         "').");
            }
            xml_paths.push_back(base_path + '/' + paths[0]);
        }
    }
    return std::make_shared<BTNodePluginClassLoader>("auto_apms_behavior_tree",
                                                     "auto_apms_behavior_tree::BTNodePluginBase",
                                                     "",
                                                     xml_paths);
}

BTCreator::BTCreator(std::shared_ptr<Factory> factory_ptr,
                     std::shared_ptr<BTNodePluginClassLoader> node_plugin_loader_ptr)
    : factory_ptr_{factory_ptr}, node_plugin_loader_ptr_{node_plugin_loader_ptr}
{
    doc_.Parse("<root BTCPP_format=\"4\"></root>");
}

BTCreator& BTCreator::RegisterNodePlugins(rclcpp::Node::SharedPtr node_ptr,
                                          const BTNodePluginManifest& node_plugin_manifest)
{
    const auto registered_nodes = GetRegisteredNodes(*factory_ptr_);
    for (const auto& [node_name, params] : node_plugin_manifest.map()) {
        // Skip node if it is already registered
        if (registered_nodes.find(node_name) != registered_nodes.end()) continue;

        // Check if the class we search for is actually available with the loader.
        if (!node_plugin_loader_ptr_->isClassAvailable(params.class_name)) {
            throw exceptions::NodeRegistrationError{
                "Node plugin '" + node_name + " (" + params.class_name +
                ")' cannot be loaded, because it's not registered by the packages searched by the plugin loader. It's "
                "also possible that you misspelled the class name in CMake when registering it in the CMakeLists.txt "
                "using auto_apms_behavior_tree_register_nodes()."};
        }

        RCLCPP_DEBUG(node_ptr->get_logger(),
                     "Registering behavior tree node plugin '%s (%s)' from library %s.",
                     node_name.c_str(),
                     params.class_name.c_str(),
                     node_plugin_loader_ptr_->getClassLibraryPath(params.class_name).c_str());

        pluginlib::UniquePtr<BTNodePluginBase> plugin_instance;
        try {
            plugin_instance = node_plugin_loader_ptr_->createUniqueInstance(params.class_name);
        } catch (const std::exception& e) {
            throw exceptions::NodeRegistrationError(
                "Failed to create an instance of node '" + node_name + " (" + params.class_name +
                ")' from plugin. Remember that the AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE macro must be called in the "
                "source file for the plugin to be discoverable. Error message: " +
                e.what() + ".");
        }

        try {
            if (plugin_instance->RequiresROSNodeParams()) {
                RosNodeParams ros_params;
                ros_params.nh = node_ptr;
                ros_params.default_port_name = params.port;
                ros_params.wait_for_server_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::duration<double>(params.wait_timeout));
                ros_params.request_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::duration<double>(params.request_timeout));
                plugin_instance->RegisterWithBehaviorTreeFactory(*factory_ptr_, node_name, &ros_params);
            }
            else {
                plugin_instance->RegisterWithBehaviorTreeFactory(*factory_ptr_, node_name);
            }
        } catch (const std::exception& e) {
            throw exceptions::NodeRegistrationError("Failed to register node '" + node_name + " (" + params.class_name +
                                                    ")' with factory: " + e.what() + ".");
        }
    }
    return *this;
}

std::unordered_map<std::string, BT::NodeType> BTCreator::GetRegisteredNodes(const Factory& factory)
{
    std::unordered_map<std::string, BT::NodeType> registered_nodes;
    for (const auto& it : factory.manifests()) registered_nodes.insert({it.first, it.second.type});
    return registered_nodes;
}

BTCreator& BTCreator::AddTreeFromXMLDocument(const tinyxml2::XMLDocument& doc)
{
    // Overwrite main tree in current document
    auto other_root = doc.RootElement();
    if (const auto name = other_root->Attribute(MAIN_TREE_ATTRIBUTE_NAME.c_str())) SetMainTreeName(name);

    auto this_root = doc_.RootElement();
    if (!other_root) throw exceptions::TreeVerificationError("Cannot merge new XMLDocument: other_root is nullptr.");

    // Iterate over all the children of the new document's root element
    for (const tinyxml2::XMLElement* child = other_root->FirstChildElement(); child != nullptr;
         child = child->NextSiblingElement()) {
        // Clone the child element to the original document
        auto copied_child = child->DeepClone(&doc_);
        // Append the copied child to the original document's root
        this_root->InsertEndChild(copied_child);
    }

    try {
        // Verify the structure of the new tree document and that all mentioned nodes are registered with the factory
        BT::VerifyXML(WriteToString(), GetRegisteredNodes(*factory_ptr_));
    } catch (const BT::RuntimeError& e) {
        throw exceptions::TreeVerificationError(e.what());
    }
    return *this;
}

BTCreator& BTCreator::AddTreeFromString(const std::string& tree_str)
{
    tinyxml2::XMLDocument new_doc;
    if (new_doc.Parse(tree_str.c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
        throw exceptions::TreeVerificationError("Cannot add tree: " + std::string(new_doc.ErrorStr()));
    }
    return AddTreeFromXMLDocument(new_doc);
}

BTCreator& BTCreator::AddTreeFromFile(const std::string& tree_file_path)
{
    tinyxml2::XMLDocument new_doc;
    if (new_doc.LoadFile(tree_file_path.c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
        throw exceptions::TreeVerificationError("Cannot add tree: " + std::string(new_doc.ErrorStr()));
    }
    return AddTreeFromXMLDocument(new_doc);
}

BTCreator& BTCreator::AddTreeFromResource(const BTResource& resource, rclcpp::Node::SharedPtr node_ptr)
{
    RegisterNodePlugins(node_ptr, BTNodePluginManifest::FromFile(resource.node_manifest_file_path));
    return AddTreeFromFile(resource.tree_file_path);
}

BT::Tree BTCreator::CreateTree(const std::string& main_tree_id, BT::Blackboard::Ptr root_blackboard_ptr)
{
    try {
        factory_ptr_->registerBehaviorTreeFromText(WriteToString());
        return factory_ptr_->createTree(main_tree_id, root_blackboard_ptr);
    } catch (const BT::RuntimeError& e) {
        throw exceptions::TreeVerificationError(e.what());
    }
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

BTCreator& BTCreator::SetMainTreeName(const std::string& main_tree_id)
{
    if (!main_tree_id.empty()) doc_.RootElement()->SetAttribute(MAIN_TREE_ATTRIBUTE_NAME.c_str(), main_tree_id.c_str());
    return *this;
}

std::string BTCreator::WriteToString() const { return WriteXMLDocumentToString(doc_); }

std::string BTCreator::WriteXMLDocumentToString(const tinyxml2::XMLDocument& doc)
{
    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);
    return printer.CStr();
}

}  // namespace auto_apms_behavior_tree