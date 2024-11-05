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

#include <algorithm>

#include "behaviortree_cpp/xml_parsing.h"
#include "auto_apms_util/resource.hpp"
#include "auto_apms_util/container.hpp"
#include "auto_apms_util/string.hpp"
#include "auto_apms_behavior_tree/exceptions.hpp"

namespace auto_apms_behavior_tree
{

TreeBuilder::TreeBuilder(std::shared_ptr<BT::BehaviorTreeFactory> factory_ptr) : factory_ptr_(factory_ptr)
{
  doc_.Parse("<root BTCPP_format=\"4\"></root>");
}

TreeBuilder& TreeBuilder::loadNodePlugins(rclcpp::Node::SharedPtr node_ptr, const NodeManifest& node_manifest,
                                          NodeRegistrationClassLoader& tree_node_loader, bool override)
{
  const auto registered_nodes = getRegisteredNodes();
  for (const auto& [node_name, params] : node_manifest.getInternalMap())
  {
    // If the node is already registered
    if (registered_nodes.find(node_name) != registered_nodes.end())
    {
      if (override)
      {
        factory_ptr_->unregisterBuilder(node_name);
      }
      else
      {
        throw exceptions::TreeBuildError("Node '" + node_name +
                                         "' is already registered. Set override = true to allow for "
                                         "overriding previously registered nodes.");
      }
    }

    // Check if the class we search for is actually available with the loader.
    if (!tree_node_loader.isClassAvailable(params.class_name))
    {
      throw exceptions::TreeBuildError("Node '" + node_name + " (Class: " + params.class_name +
                                       ")' cannot be loaded, because the class name is not known to the class loader. "
                                       "Make sure that it's spelled correctly and registered by calling "
                                       "auto_apms_behavior_tree_register_nodes() in the CMakeLists.txt of the "
                                       "corresponding package.");
    }

    RCLCPP_DEBUG(node_ptr->get_logger(), "Loading behavior tree node '%s' (Class: %s) from library %s.",
                 node_name.c_str(), params.class_name.c_str(),
                 tree_node_loader.getClassLibraryPath(params.class_name).c_str());

    pluginlib::UniquePtr<NodeRegistrationInterface> plugin_instance;
    try
    {
      plugin_instance = tree_node_loader.createUniqueInstance(params.class_name);
    }
    catch (const std::exception& e)
    {
      throw exceptions::TreeBuildError("Failed to create an instance of node '" + node_name +
                                       " (Class: " + params.class_name +
                                       ")'. Remember that the AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE "
                                       "macro must be called in the source file for the node class to be discoverable. "
                                       "Error message: " +
                                       e.what() + ".");
    }

    try
    {
      if (plugin_instance->requiresROSNodeParams())
      {
        RosNodeContext ros_node_context(node_ptr, params);
        plugin_instance->registerWithBehaviorTreeFactory(*factory_ptr_, node_name, &ros_node_context);
      }
      else
      {
        plugin_instance->registerWithBehaviorTreeFactory(*factory_ptr_, node_name);
      }
    }
    catch (const std::exception& e)
    {
      throw exceptions::TreeBuildError("Failed to register node '" + node_name + " (Class: " + params.class_name +
                                       ")': " + e.what() + ".");
    }
  }
  return *this;
}

TreeBuilder& TreeBuilder::loadNodePlugins(rclcpp::Node::SharedPtr node_ptr, const NodeManifest& node_manifest,
                                          bool override)
{
  NodeRegistrationClassLoader loader;
  return loadNodePlugins(node_ptr, node_manifest, loader, override);
}

TreeBuilder& TreeBuilder::mergeTreesFromDocument(const tinyxml2::XMLDocument& doc)
{
  const tinyxml2::XMLElement* other_root = doc.RootElement();
  if (!other_root)
    throw exceptions::TreeBuildError("Cannot merge trees: other_root is nullptr.");
  if (strcmp(other_root->Name(), ROOT_ELEMENT_NAME) != 0)
    throw exceptions::TreeBuildError("Cannot merge trees: Root element of other document is not named '" +
                                     std::string(ROOT_ELEMENT_NAME) + "'.");
  // Overwrite main tree in current document
  if (const char* name = other_root->Attribute(MAIN_TREE_ATTRIBUTE_NAME))
    setMainTreeName(name);

  const auto this_tree_names = getAllTreeNames();
  const auto other_tree_names = getAllTreeNames(doc);

  // Verify that there are no duplicate tree names
  const auto common_tree_names = auto_apms_util::haveCommonElements(this_tree_names, other_tree_names);
  if (!common_tree_names.empty())
  {
    throw exceptions::TreeBuildError("Cannot merge trees: The following trees are already defined: " +
                                     rcpputils::join(common_tree_names, ", ") + ".");
  }

  // Iterate over all the children of other root
  const tinyxml2::XMLElement* child = other_root->FirstChildElement(TREE_ELEMENT_NAME);
  if (!child)
  {
    throw exceptions::TreeBuildError("Cannot merge trees: Other document has no children named '" +
                                     std::string(TREE_ELEMENT_NAME) + "'.");
  }
  for (; child != nullptr; child = child->NextSiblingElement())
  {
    auto copied_child = child->DeepClone(&doc_);       // Clone the child element to this document
    doc_.RootElement()->InsertEndChild(copied_child);  // Append the copied child to this document's root
  }
  return *this;
}

TreeBuilder& TreeBuilder::mergeTreesFromString(const std::string& tree_str)
{
  tinyxml2::XMLDocument new_doc;
  if (new_doc.Parse(tree_str.c_str()) != tinyxml2::XMLError::XML_SUCCESS)
  {
    throw exceptions::TreeBuildError("Cannot add tree: " + std::string(new_doc.ErrorStr()));
  }
  return mergeTreesFromDocument(new_doc);
}

TreeBuilder& TreeBuilder::mergeTreesFromFile(const std::string& tree_file_path)
{
  tinyxml2::XMLDocument new_doc;
  if (new_doc.LoadFile(tree_file_path.c_str()) != tinyxml2::XMLError::XML_SUCCESS)
  {
    throw exceptions::TreeBuildError("Cannot add tree: " + std::string(new_doc.ErrorStr()));
  }
  return mergeTreesFromDocument(new_doc);
}

TreeBuilder& TreeBuilder::mergeTreesFromResource(const TreeResource& resource, rclcpp::Node::SharedPtr node_ptr)
{
  loadNodePlugins(node_ptr, NodeManifest::fromFile(resource.node_manifest_file_path));
  return mergeTreesFromFile(resource.tree_file_path);
}

bool TreeBuilder::isExistingTreeName(const std::string& tree_name)
{
  const auto all_names = getAllTreeNames();
  if (std::find(all_names.begin(), all_names.end(), tree_name) == all_names.end())
    return false;
  return true;
}

tinyxml2::XMLElement* TreeBuilder::insertNewTreeElement(const std::string& tree_name)
{
  if (isExistingTreeName(tree_name))
  {
    throw exceptions::TreeBuildError("Cannot insert a tree element with name '" + tree_name +
                                     "' because it already existing.");
  }
  if (tinyxml2::XMLElement* ele = doc_.RootElement()->InsertNewChildElement(TREE_ELEMENT_NAME))
  {
    ele->SetAttribute(TREE_NAME_ATTRIBUTE_NAME, tree_name.c_str());
    return ele;
  }
  throw exceptions::TreeBuildError("Root element has no children named '" + std::string(TREE_ELEMENT_NAME) + "'.");
}

tinyxml2::XMLElement* TreeBuilder::getTreeElement(const std::string& tree_name)
{
  if (!isExistingTreeName(tree_name))
  {
    throw exceptions::TreeBuildError("Cannot get tree element with name '" + tree_name + "' because it doesn't exist.");
  }
  tinyxml2::XMLElement* ele = doc_.RootElement()->FirstChildElement(TREE_ELEMENT_NAME);
  while (ele && !ele->Attribute(TREE_NAME_ATTRIBUTE_NAME, tree_name.c_str()))
  {
    ele = ele->NextSiblingElement();
  }
  return ele;
}

std::vector<std::string> TreeBuilder::getAllTreeNames() const
{
  return getAllTreeNames(doc_);
}

std::string TreeBuilder::getMainTreeName() const
{
  if (const auto main_tree_name = doc_.RootElement()->Attribute(MAIN_TREE_ATTRIBUTE_NAME))
    return main_tree_name;
  return "";
}

TreeBuilder& TreeBuilder::setMainTreeName(const std::string& main_tree_name)
{
  if (!main_tree_name.empty())
    doc_.RootElement()->SetAttribute(MAIN_TREE_ATTRIBUTE_NAME, main_tree_name.c_str());
  return *this;
}

std::string TreeBuilder::writeTreeDocumentToString() const
{
  return writeTreeDocumentToString(doc_);
}

std::unordered_map<std::string, BT::NodeType> TreeBuilder::getRegisteredNodes() const
{
  std::unordered_map<std::string, BT::NodeType> registered_nodes;
  for (const auto& it : factory_ptr_->manifests())
    registered_nodes.insert({ it.first, it.second.type });
  return registered_nodes;
}

bool TreeBuilder::verifyTreeDocument() const
{
  try
  {
    BT::VerifyXML(writeTreeDocumentToString(), getRegisteredNodes());
  }
  catch (const BT::RuntimeError& e)
  {
    return false;
  }
  return true;
}

Tree TreeBuilder::buildTree(const std::string main_tree_name, TreeBlackboardSharedPtr root_bb_ptr)
{
  Tree tree;
  try
  {
    factory_ptr_->registerBehaviorTreeFromText(writeTreeDocumentToString());
    tree = factory_ptr_->createTree(main_tree_name, root_bb_ptr);
    factory_ptr_->clearRegisteredBehaviorTrees();
  }
  catch (const std::exception& e)
  {
    throw exceptions::TreeBuildError("Failed to build tree. " + std::string(e.what()));
  }
  return tree;
}

Tree TreeBuilder::buildTree(TreeBlackboardSharedPtr root_bb_ptr)
{
  return buildTree("", root_bb_ptr);
}

std::vector<std::string> TreeBuilder::getAllTreeNames(const tinyxml2::XMLDocument& doc)
{
  std::vector<std::string> names;
  if (const auto root = doc.RootElement())
  {
    for (const tinyxml2::XMLElement* child = root->FirstChildElement(); child != nullptr;
         child = child->NextSiblingElement())
    {
      if (strcmp(TREE_ELEMENT_NAME, child->Name()) == 0)
      {
        if (const auto name = child->Attribute(TREE_NAME_ATTRIBUTE_NAME))
        {
          names.push_back(name);
        }
        else
        {
          throw exceptions::TreeBuildError("Cannot get tree name, because required attribute '" +
                                           std::string(TREE_NAME_ATTRIBUTE_NAME) + "' is missing.");
        }
      }
    }
  }
  return names;
}

std::string TreeBuilder::writeTreeDocumentToString(const tinyxml2::XMLDocument& doc)
{
  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  return printer.CStr();
}

}  // namespace auto_apms_behavior_tree