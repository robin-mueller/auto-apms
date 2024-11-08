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

#include "auto_apms_behavior_tree/creator/tree_builder.hpp"

#include <regex>

#include "auto_apms_behavior_tree/exceptions.hpp"
#include "auto_apms_util/container.hpp"
#include "auto_apms_util/resource.hpp"
#include "auto_apms_util/string.hpp"
#include "behaviortree_cpp/xml_parsing.h"

namespace auto_apms_behavior_tree
{

TreeBuilder::TreeBuilder(
  rclcpp::Node::SharedPtr node_ptr, std::shared_ptr<NodeRegistrationClassLoader> tree_node_loader_ptr)
: node_wptr_(node_ptr),
  tree_node_loader_ptr_(tree_node_loader_ptr)
{
  ElementPtr root_ele = doc_.NewElement(ROOT_ELEMENT_NAME);
  root_ele->SetAttribute("BTCPP_format", "4");
  doc_.InsertFirstChild(root_ele);
}

TreeBuilder & TreeBuilder::setScriptingEnum(const std::string & enum_name, int val)
{
  factory_.registerScriptingEnum(enum_name, val);
  return *this;
}

TreeBuilder & TreeBuilder::loadNodePlugins(const NodeManifest & node_manifest, bool override)
{
  rclcpp::Node::SharedPtr node_ptr = node_wptr_.lock();
  if (!node_ptr) {
    throw exceptions::TreeBuildError("Cannot load nodes because node_ptr is null.");
  }

  const auto registered_nodes = getRegisteredNodesTypeMap();
  for (const auto & [node_name, params] : node_manifest.getInternalMap()) {
    // If the node is already registered
    if (registered_nodes.find(node_name) != registered_nodes.end()) {
      if (override) {
        factory_.unregisterBuilder(node_name);
      } else {
        throw exceptions::TreeBuildError(
          "Node '" + node_name +
          "' is already registered. Set override = true to allow for "
          "overriding previously registered nodes.");
      }
    }

    // Check if the class we search for is actually available with the loader.
    if (!tree_node_loader_ptr_->isClassAvailable(params.class_name)) {
      throw exceptions::TreeBuildError(
        "Node '" + node_name + " (Class: " + params.class_name +
        ")' cannot be loaded, because the class name is not known to the class loader. "
        "Make sure that it's spelled correctly and registered by calling "
        "auto_apms_behavior_tree_register_nodes() in the CMakeLists.txt of the "
        "corresponding package.");
    }

    RCLCPP_DEBUG(
      node_ptr->get_logger(), "Loading behavior tree node '%s' (Class: %s) from library %s.", node_name.c_str(),
      params.class_name.c_str(), tree_node_loader_ptr_->getClassLibraryPath(params.class_name).c_str());

    pluginlib::UniquePtr<NodeRegistrationInterface> plugin_instance;
    try {
      plugin_instance = tree_node_loader_ptr_->createUniqueInstance(params.class_name);
    } catch (const std::exception & e) {
      throw exceptions::TreeBuildError(
        "Failed to create an instance of node '" + node_name + " (Class: " + params.class_name +
        ")'. Remember that the AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE "
        "macro must be called in the source file for the node class to be discoverable. "
        "Error message: " +
        e.what() + ".");
    }

    try {
      if (plugin_instance->requiresROSNodeParams()) {
        RosNodeContext ros_node_context(node_ptr, params);
        plugin_instance->registerWithBehaviorTreeFactory(factory_, node_name, &ros_node_context);
      } else {
        plugin_instance->registerWithBehaviorTreeFactory(factory_, node_name);
      }
    } catch (const std::exception & e) {
      throw exceptions::TreeBuildError(
        "Failed to register node '" + node_name + " (Class: " + params.class_name + ")': " + e.what() + ".");
    }
  }
  return *this;
}

std::unordered_map<std::string, BT::NodeType> TreeBuilder::getRegisteredNodesTypeMap() const
{
  std::unordered_map<std::string, BT::NodeType> map;
  for (const auto & it : factory_.manifests()) map.insert({it.first, it.second.type});
  return map;
}

std::vector<std::string> TreeBuilder::getRegisteredNodes() const
{
  std::vector<std::string> vec;
  for (const auto & it : factory_.manifests()) vec.push_back(it.first);
  return vec;
}

TreeBuilder & TreeBuilder::mergeTreesFromDocument(const Document & doc)
{
  ConstElementPtr other_root = doc.RootElement();
  if (!other_root) throw exceptions::TreeBuildError("Cannot merge trees: other_root is nullptr.");
  if (strcmp(other_root->Name(), ROOT_ELEMENT_NAME) != 0)
    throw exceptions::TreeBuildError(
      "Cannot merge trees: Root element of other document is not named '" + std::string(ROOT_ELEMENT_NAME) + "'.");
  // Overwrite main tree in current document
  if (const char * name = other_root->Attribute(MAIN_TREE_ATTRIBUTE_NAME)) setMainTreeName(name);

  const auto this_tree_names = getAllTreeNames();
  const auto other_tree_names = getAllTreeNames(doc);

  // Verify that there are no duplicate tree names
  const auto common_tree_names = auto_apms_util::haveCommonElements(this_tree_names, other_tree_names);
  if (!common_tree_names.empty()) {
    throw exceptions::TreeBuildError(
      "Cannot merge trees: The following trees are already defined: " + rcpputils::join(common_tree_names, ", ") + ".");
  }

  // Iterate over all the children of other root
  ConstElementPtr child = other_root->FirstChildElement();  // Allow include tags
  if (!child) {
    throw exceptions::TreeBuildError("Cannot merge trees: Other document has no children.");
  }
  for (; child != nullptr; child = child->NextSiblingElement()) {
    auto copied_child = child->DeepClone(&doc_);       // Clone the child element to this document
    doc_.RootElement()->InsertEndChild(copied_child);  // Append the copied child to this document's root
  }
  return *this;
}

TreeBuilder & TreeBuilder::mergeTreesFromString(const std::string & tree_str)
{
  Document new_doc;
  if (new_doc.Parse(tree_str.c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
    throw exceptions::TreeBuildError("Cannot add tree: " + std::string(new_doc.ErrorStr()));
  }
  return mergeTreesFromDocument(new_doc);
}

TreeBuilder & TreeBuilder::mergeTreesFromFile(const std::string & tree_file_path)
{
  Document new_doc;
  if (new_doc.LoadFile(tree_file_path.c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
    throw exceptions::TreeBuildError("Cannot add tree: " + std::string(new_doc.ErrorStr()));
  }
  return mergeTreesFromDocument(new_doc);
}

TreeBuilder & TreeBuilder::mergeTreesFromResource(const TreeResource & resource)
{
  loadNodePlugins(NodeManifest::fromFile(resource.node_manifest_file_path));
  return mergeTreesFromFile(resource.tree_file_path);
}

TreeBuilder::ElementPtr TreeBuilder::insertNewTreeElement(const std::string & tree_name)
{
  if (isExistingTreeName(tree_name)) {
    throw exceptions::TreeBuildError(
      "Cannot insert a tree element with name '" + tree_name + "' because it is already existing.");
  }
  if (ElementPtr ele = doc_.RootElement()->InsertNewChildElement(TREE_ELEMENT_NAME)) {
    ele->SetAttribute(TREE_NAME_ATTRIBUTE_NAME, tree_name.c_str());
    return ele;
  }
  throw exceptions::TreeBuildError("Root element has no children named '" + std::string(TREE_ELEMENT_NAME) + "'.");
}

TreeBuilder::ElementPtr TreeBuilder::insertNewNodeElement(
  ElementPtr parent_element, const std::string & node_name, const NodeRegistrationParams & registration_params)
{
  if (!auto_apms_util::contains(getRegisteredNodes(), node_name) && !registration_params.class_name.empty()) {
    // Try to load the node if it isn't registered yet and registration_params was given with a non-empty class name
    loadNodePlugins(NodeManifest({{node_name, registration_params}}));
  }
  ElementPtr ele = parent_element->InsertNewChildElement(node_name.c_str());
  addNodePortValues(ele);  // Insert default values
  return ele;
}

TreeBuilder & TreeBuilder::addNodePortValues(ElementPtr node_element, PortValues port_values, bool verify)
{
  const char * node_name = node_element->Name();
  if (strcmp(node_name, TREE_ELEMENT_NAME) == 0) {
    throw exceptions::TreeBuildError(
      "Cannot add default node ports to element with name '" + std::string(node_name) + "'. Must not be a <" +
      std::string(TREE_ELEMENT_NAME) + "> element.");
  }

  // Load tree nodes model
  Document model_doc;
  if (model_doc.Parse(BT::writeTreeNodesModelXML(factory_, true).c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
    throw exceptions::TreeBuildError(
      "Error parsing the current tree nodes model: " + std::string(model_doc.ErrorStr()));
  }

  // Extract the port names and default values from the model
  const char model_ele_name[] = "TreeNodesModel";
  ConstElementPtr model_ele = model_doc.RootElement()->FirstChildElement(model_ele_name);
  if (!model_ele) {
    throw exceptions::TreeBuildError("Element <" + std::string(model_ele_name) + "> doesn't exist.");
  }
  ConstElementPtr node_ele = model_ele->FirstChildElement();
  while (node_ele && !node_ele->Attribute("ID", node_name)) {
    node_ele = node_ele->NextSiblingElement();
  }
  if (!node_ele) {
    throw exceptions::TreeBuildError(
      "Cannot add default node ports to '" + std::string(node_name) +
      "' because the node wasn't found in tree nodes model.");
  }
  std::vector<std::string> port_names;
  PortValues default_values;
  for (ConstElementPtr port_ele = node_ele->FirstChildElement(); port_ele != nullptr;
       port_ele = port_ele->NextSiblingElement()) {
    if (std::regex_match(port_ele->Name(), std::regex("input_port|output_port|inout_port"))) {
      const char * port_name = port_ele->Attribute("name");
      const char * port_default = port_ele->Attribute("default");
      port_names.push_back(port_name);
      if (port_name && port_default) {
        default_values[port_name] = port_default;
      }
    }
  }

  // Verify port_values
  if (verify) {
    std::vector<std::string> unkown_keys;
    for (const auto & [key, val] : port_values) {
      if (!auto_apms_util::contains(port_names, key)) unkown_keys.push_back(key);
    }
    if (!unkown_keys.empty()) {
      throw exceptions::TreeBuildError(
        "Verification of argument port_values failed. The following keys are not associated with node '" +
        std::string(node_name) + "': [" + rcpputils::join(unkown_keys, ", ") + "].");
    }
  }

  // Populate the node's attributes
  for (const auto & name : port_names) {
    if (port_values.find(name) != port_values.end()) {
      node_element->SetAttribute(name.c_str(), port_values[name].c_str());
    } else if (default_values.find(name) != default_values.end()) {
      node_element->SetAttribute(name.c_str(), default_values[name].c_str());
    }
    // If key neither exists in port_values nor in default_values, don't set port attribute (has_value() will be false)
  }

  return *this;
}

bool TreeBuilder::isExistingTreeName(const std::string & tree_name)
{
  if (auto_apms_util::contains(getAllTreeNames(), tree_name)) return true;
  return false;
}

TreeBuilder::ElementPtr TreeBuilder::getTreeElement(const std::string & tree_name)
{
  if (!isExistingTreeName(tree_name)) {
    throw exceptions::TreeBuildError("Cannot get tree element with name '" + tree_name + "' because it doesn't exist.");
  }
  ElementPtr ele = doc_.RootElement()->FirstChildElement(TREE_ELEMENT_NAME);
  while (ele && !ele->Attribute(TREE_NAME_ATTRIBUTE_NAME, tree_name.c_str())) {
    ele = ele->NextSiblingElement();
  }
  return ele;
}

std::vector<std::string> TreeBuilder::getAllTreeNames() const { return getAllTreeNames(doc_); }

std::string TreeBuilder::getMainTreeName() const
{
  if (const auto main_tree_name = doc_.RootElement()->Attribute(MAIN_TREE_ATTRIBUTE_NAME)) return main_tree_name;
  return "";
}

TreeBuilder & TreeBuilder::setMainTreeName(const std::string & main_tree_name)
{
  if (!main_tree_name.empty()) doc_.RootElement()->SetAttribute(MAIN_TREE_ATTRIBUTE_NAME, main_tree_name.c_str());
  return *this;
}

std::string TreeBuilder::writeTreeDocumentToString() const { return writeTreeDocumentToString(doc_); }

bool TreeBuilder::verifyTree() const
{
  try {
    BT::VerifyXML(writeTreeDocumentToString(), getRegisteredNodesTypeMap());
  } catch (const BT::RuntimeError & e) {
    return false;
  }
  return true;
}

Tree TreeBuilder::buildTree(const std::string main_tree_name, TreeBlackboardSharedPtr root_bb_ptr)
{
  Tree tree;
  try {
    factory_.registerBehaviorTreeFromText(writeTreeDocumentToString());
    tree = factory_.createTree(main_tree_name, root_bb_ptr);
    factory_.clearRegisteredBehaviorTrees();
  } catch (const std::exception & e) {
    throw exceptions::TreeBuildError("Error during buildTree(): " + std::string(e.what()));
  }
  return tree;
}

Tree TreeBuilder::buildTree(TreeBlackboardSharedPtr root_bb_ptr) { return buildTree("", root_bb_ptr); }

std::vector<std::string> TreeBuilder::getAllTreeNames(const Document & doc)
{
  std::vector<std::string> names;
  if (const auto root = doc.RootElement()) {
    for (ConstElementPtr child = root->FirstChildElement(); child != nullptr; child = child->NextSiblingElement()) {
      if (strcmp(TREE_ELEMENT_NAME, child->Name()) == 0) {
        if (const auto name = child->Attribute(TREE_NAME_ATTRIBUTE_NAME)) {
          names.push_back(name);
        } else {
          throw exceptions::TreeBuildError(
            "Cannot get tree name, because required attribute '" + std::string(TREE_NAME_ATTRIBUTE_NAME) +
            "' is missing.");
        }
      }
    }
  }
  return names;
}

std::string TreeBuilder::writeTreeDocumentToString(const Document & doc)
{
  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  return printer.CStr();
}

}  // namespace auto_apms_behavior_tree