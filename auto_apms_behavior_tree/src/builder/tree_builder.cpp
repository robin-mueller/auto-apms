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

#include "auto_apms_behavior_tree/builder/tree_builder.hpp"

#include "behaviortree_cpp/xml_parsing.h"
#include "auto_apms_core/resources.hpp"
#include "auto_apms_behavior_tree/exceptions.hpp"

namespace auto_apms_behavior_tree
{

TreeBuilder::TreeBuilder()
{
  doc_.Parse("<root BTCPP_format=\"4\"></root>");
}

TreeBuilder& TreeBuilder::registerNodePlugins(rclcpp::Node::SharedPtr node_ptr,
                                              const NodeManifest& node_plugin_manifest,
                                              NodePluginClassLoader& tree_node_loader, bool override)
{
  const auto registered_nodes = getRegisteredNodes();
  for (const auto& [node_name, params] : node_plugin_manifest.getInternalMap())
  {
    // If the node is already registered
    if (registered_nodes.find(node_name) != registered_nodes.end())
    {
      if (!override)
      {
        throw exceptions::TreeBuildError("Node '" + node_name +
                                         "' is already registered. Set override = true to allow for "
                                         "overriding previously registered nodes.");
      }
      impl_.unregisterBuilder(node_name);
    }

    // Check if the class we search for is actually available with the loader.
    if (!tree_node_loader.isClassAvailable(params.class_name))
    {
      throw exceptions::TreeBuildError("Node plugin '" + node_name + " (" + params.class_name +
                                       ")' cannot be loaded, because it's not registered by the packages "
                                       "searched by the plugin loader. It's "
                                       "also possible that you misspelled the class name in CMake when "
                                       "registering it in the CMakeLists.txt "
                                       "using auto_apms_behavior_tree_register_nodes().");
    }

    RCLCPP_DEBUG(node_ptr->get_logger(), "Loading behavior tree node plugin '%s (%s)' from library %s.",
                 node_name.c_str(), params.class_name.c_str(),
                 tree_node_loader.getClassLibraryPath(params.class_name).c_str());

    pluginlib::UniquePtr<NodeRegistrationInterface> plugin_instance;
    try
    {
      plugin_instance = tree_node_loader.createUniqueInstance(params.class_name);
    }
    catch (const std::exception& e)
    {
      throw exceptions::TreeBuildError("Failed to create an instance of node '" + node_name + " (" + params.class_name +
                                       ")' from plugin. Remember that the AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE "
                                       "macro must be called in the "
                                       "source file for the plugin to be discoverable. Error message: " +
                                       e.what() + ".");
    }

    try
    {
      if (plugin_instance->requiresROSNodeParams())
      {
        RosNodeParams ros_params;
        ros_params.nh = node_ptr;
        ros_params.default_port_name = params.port;
        ros_params.wait_for_server_timeout =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(params.wait_timeout));
        ros_params.request_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(params.request_timeout));
        plugin_instance->registerWithBehaviorTreeFactory(impl_, node_name, &ros_params);
      }
      else
      {
        plugin_instance->registerWithBehaviorTreeFactory(impl_, node_name);
      }
    }
    catch (const std::exception& e)
    {
      throw exceptions::TreeBuildError("Failed to register node '" + node_name + " (" + params.class_name +
                                       ")' with factory: " + e.what() + ".");
    }
  }
  return *this;
}

TreeBuilder& TreeBuilder::registerNodePlugins(rclcpp::Node::SharedPtr node_ptr,
                                              const NodeManifest& node_plugin_manifest, bool override)
{
  NodePluginClassLoader loader;
  return registerNodePlugins(node_ptr, node_plugin_manifest, loader, override);
}

TreeBuilder& TreeBuilder::addTreeFromXMLDocument(const tinyxml2::XMLDocument& doc)
{
  // Overwrite main tree in current document
  auto other_root = doc.RootElement();
  if (!other_root)
    throw exceptions::TreeBuildError("Cannot merge tree document: other_root is nullptr.");
  if (const auto name = other_root->Attribute(MAIN_TREE_ATTRIBUTE_NAME.c_str()))
    setMainTreeName(name);

  const auto this_tree_names = getTreeNames(doc_);
  const auto other_tree_names = getTreeNames(doc);

  // Verify that there are no duplicate tree names
  std::vector<std::string> same_names;
  std::set_intersection(this_tree_names.begin(), this_tree_names.end(), other_tree_names.begin(),
                        other_tree_names.end(), std::inserter(same_names, same_names.begin()));
  if (!same_names.empty())
  {
    std::ostringstream oss;
    std::copy(same_names.begin(), same_names.end() - 1, std::ostream_iterator<std::string>(oss, ", "));
    oss << same_names.back();
    throw exceptions::TreeBuildError(
        "Cannot merge tree document: The following trees are already defined: " + oss.str() + ".");
  }

  // Iterate over all the children of the new document's root element
  auto this_root = doc_.RootElement();
  for (const tinyxml2::XMLElement* child = other_root->FirstChildElement(); child != nullptr;
       child = child->NextSiblingElement())
  {
    // Clone the child element to the original document
    auto copied_child = child->DeepClone(&doc_);
    // Append the copied child to the original document's root
    this_root->InsertEndChild(copied_child);
  }

  try
  {
    // Verify the structure of the new tree document and that all mentioned nodes are registered with the factory
    BT::VerifyXML(writeTreeBufferToString(), getRegisteredNodes());
  }
  catch (const BT::RuntimeError& e)
  {
    throw exceptions::TreeBuildError(e.what());
  }
  return *this;
}

TreeBuilder& TreeBuilder::addTreeFromString(const std::string& tree_str)
{
  tinyxml2::XMLDocument new_doc;
  if (new_doc.Parse(tree_str.c_str()) != tinyxml2::XMLError::XML_SUCCESS)
  {
    throw exceptions::TreeBuildError("Cannot add tree: " + std::string(new_doc.ErrorStr()));
  }
  return addTreeFromXMLDocument(new_doc);
}

TreeBuilder& TreeBuilder::addTreeFromFile(const std::string& tree_file_path)
{
  tinyxml2::XMLDocument new_doc;
  if (new_doc.LoadFile(tree_file_path.c_str()) != tinyxml2::XMLError::XML_SUCCESS)
  {
    throw exceptions::TreeBuildError("Cannot add tree: " + std::string(new_doc.ErrorStr()));
  }
  return addTreeFromXMLDocument(new_doc);
}

TreeBuilder& TreeBuilder::addTreeFromResource(const TreeResource& resource, rclcpp::Node::SharedPtr node_ptr)
{
  registerNodePlugins(node_ptr, NodeManifest::fromFile(resource.node_manifest_file_path));
  return addTreeFromFile(resource.tree_file_path);
}

std::string TreeBuilder::getMainTreeName() const
{
  if (const auto main_tree_name = doc_.RootElement()->Attribute(MAIN_TREE_ATTRIBUTE_NAME.c_str()))
    return main_tree_name;
  return "";
}

TreeBuilder& TreeBuilder::setMainTreeName(const std::string& main_tree_name)
{
  if (!main_tree_name.empty())
    doc_.RootElement()->SetAttribute(MAIN_TREE_ATTRIBUTE_NAME.c_str(), main_tree_name.c_str());
  return *this;
}

std::string TreeBuilder::writeTreeBufferToString() const
{
  return writeXMLDocumentToString(doc_);
}

std::unordered_map<std::string, BT::NodeType> TreeBuilder::getRegisteredNodes()
{
  std::unordered_map<std::string, BT::NodeType> registered_nodes;
  for (const auto& it : impl_.manifests())
    registered_nodes.insert({ it.first, it.second.type });
  return registered_nodes;
}

Tree TreeBuilder::getTree(const std::string main_tree_name, TreeBlackboardSharedPtr root_bb_ptr)
{
  setMainTreeName(main_tree_name);
  Tree tree;
  try
  {
    impl_.registerBehaviorTreeFromText(writeTreeBufferToString());
    tree = impl_.createTree(getMainTreeName(), root_bb_ptr);
    impl_.clearRegisteredBehaviorTrees();
  }
  catch (const std::exception& e)
  {
    throw exceptions::TreeBuildError(e.what());
  }
  return tree;
}

Tree TreeBuilder::getTree(TreeBlackboardSharedPtr root_bb_ptr)
{
  return getTree("", root_bb_ptr);
}

std::set<std::string> TreeBuilder::getTreeNames(const tinyxml2::XMLDocument& doc)
{
  std::set<std::string> names;
  if (const auto root = doc.RootElement())
  {
    for (const tinyxml2::XMLElement* child = root->FirstChildElement(); child != nullptr;
         child = child->NextSiblingElement())
    {
      if (TREE_ELEMENT_NAME.compare(child->Name()) == 0)
      {
        if (const auto name = child->Attribute(TREE_ID_ATTRIBUTE_NAME.c_str()))
        {
          names.insert(name);
        }
        else
        {
          throw exceptions::TreeBuildError("Cannot get tree name, because required attribute '" +
                                           TREE_ID_ATTRIBUTE_NAME + "' is missing.");
        }
      }
    }
  }
  return names;
}

std::string TreeBuilder::writeXMLDocumentToString(const tinyxml2::XMLDocument& doc)
{
  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  return printer.CStr();
}

}  // namespace auto_apms_behavior_tree