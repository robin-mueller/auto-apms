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

#include "auto_apms_behavior_tree_core/builder.hpp"

#include <regex>

#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_util/container.hpp"
#include "auto_apms_util/resource.hpp"
#include "auto_apms_util/string.hpp"
#include "behaviortree_cpp/xml_parsing.h"

namespace auto_apms_behavior_tree::core
{

std::set<std::string> getNativeNodeNames()
{
  std::set<std::string> names;
  BT::BehaviorTreeFactory default_factory;
  for (const auto & [name, _] : default_factory.manifests()) {
    names.insert(name);
  }
  return names;
}

TreeBuilder::TreeBuilder(
  rclcpp::Node::SharedPtr ros_node, rclcpp::CallbackGroup::SharedPtr tree_node_callback_group,
  rclcpp::executors::SingleThreadedExecutor::SharedPtr tree_node_waitables_executor,
  NodeRegistrationLoader::SharedPtr tree_node_loader)
: ros_node_wptr_(ros_node),
  tree_node_waitables_callback_group_wptr_(tree_node_callback_group),
  tree_node_waitables_executor_wptr_(tree_node_waitables_executor),
  tree_node_loader_ptr_(tree_node_loader),
  logger_(rclcpp::get_logger(LOGGER_NAME)),
  doc_(),
  factory_(),
  all_node_classes_package_map_(auto_apms_behavior_tree::core::NodeRegistrationLoader().getClassPackageMap()),
  native_node_names_(getNativeNodeNames())
{
  logger_ = ros_node->get_logger().get_child(LOGGER_NAME);
}

TreeBuilder::TreeBuilder(NodeRegistrationLoader::SharedPtr tree_node_loader)
: TreeBuilder(
    std::shared_ptr<rclcpp::Node>(), std::shared_ptr<rclcpp::CallbackGroup>(),
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>(), tree_node_loader)
{
}

TreeBuilder & TreeBuilder::mergeTreeDocument(const TreeDocument & other, bool adopt_root_tree)
{
  doc_.merge(other, adopt_root_tree);
  return *this;
}

TreeBuilder::TreeElement TreeBuilder::newTree(const std::string & tree_name)
{
  if (tree_name.empty()) {
    throw exceptions::TreeBuildError("Cannot create a new tree with an empty name");
  }
  if (doc_.isExistingTreeName(tree_name)) {
    throw exceptions::TreeBuildError(
      "Cannot create a new tree with name '" + tree_name + "' because it already exists.");
  }
  TreeDocument::XMLElement * new_ele = doc_.RootElement()->InsertNewChildElement(TreeDocument::TREE_ELEMENT_NAME);
  new_ele->SetAttribute(TreeDocument::TREE_NAME_ATTRIBUTE_NAME, tree_name.c_str());
  return TreeElement(this, new_ele);
}

TreeBuilder::TreeElement TreeBuilder::newTreeFromDocument(const TreeDocument & other, const std::string & tree_name)
{
  std::string name(tree_name);
  if (name.empty()) {
    if (other.hasRootTreeName()) {
      name = other.getRootTreeName();
    } else {
      throw exceptions::TreeBuildError(
        "Failed to create new tree element from another document because argument tree_name was omitted and the "
        "builder cannot determine it automatically, since the required attribute '" +
        std::string(TreeDocument::ROOT_TREE_ATTRIBUTE_NAME) + "' was not specified.");
    }
  }
  TreeElement tree_ele = newTree(name);
  tree_ele.insertTreeFromDocument(other, name);
  return tree_ele;
}

TreeBuilder::TreeElement TreeBuilder::newTreeFromString(const std::string & tree_str, const std::string & tree_name)
{
  TreeDocument new_doc;
  new_doc.mergeString(tree_str, true);
  return newTreeFromDocument(new_doc, tree_name);
}

TreeBuilder::TreeElement TreeBuilder::newTreeFromFile(const std::string & path, const std::string & tree_name)
{
  TreeDocument new_doc;
  new_doc.mergeFile(path, true);
  return newTreeFromDocument(new_doc, tree_name);
}

TreeBuilder::TreeElement TreeBuilder::newTreeFromResource(const TreeResource & resource, const std::string & tree_name)
{
  TreeDocument new_doc;
  new_doc.mergeFile(resource.tree_file_path_, true);
  makeNodesAvailable(resource.getNodeManifest(), false);
  return newTreeFromDocument(new_doc, tree_name);
}

bool TreeBuilder::hasTree(const std::string & tree_name) { return doc_.isExistingTreeName(tree_name); }

TreeBuilder::TreeElement TreeBuilder::getTree(const std::string & tree_name)
{
  return TreeElement(this, doc_.getXMLElementForTreeWithName(tree_name));
}

TreeBuilder & TreeBuilder::setRootTreeName(const std::string & tree_name)
{
  doc_.setRootTreeName(tree_name);
  return *this;
}

TreeBuilder & TreeBuilder::setRootTreeName(const TreeElement & tree)
{
  doc_.setRootTreeName(tree.getName());
  return *this;
}

bool TreeBuilder::hasRootTree() { return doc_.hasRootTreeName(); }

TreeBuilder::TreeElement TreeBuilder::getRootTree() { return getTree(doc_.getRootTreeName()); }

TreeBuilder & TreeBuilder::removeTree(const std::string & tree_name)
{
  doc_.removeTreeWithName(tree_name);
  return *this;
}

TreeBuilder & TreeBuilder::removeTree(const TreeElement & tree)
{
  doc_.removeTreeWithName(tree.getName());
  return *this;
}

std::vector<std::string> TreeBuilder::getAllTreeNames() const { return doc_.getAllTreeNames(); }

std::string TreeBuilder::writeTreeDocumentToString() const { return doc_.str(); }

TreeBuilder & TreeBuilder::setScriptingEnum(const std::string & enum_name, int val)
{
  factory_.registerScriptingEnum(enum_name, val);
  return *this;
}

TreeBuilder & TreeBuilder::makeNodesAvailable(const NodeManifest & tree_node_manifest, bool override)
{
  // Make sure that there are no node names that are reserved for native nodes
  std::set<std::string> all_registration_names;
  for (const auto & [name, _] : tree_node_manifest.getInternalMap()) all_registration_names.insert(name);
  if (const std::set<std::string> common =
        auto_apms_util::getCommonElements(all_registration_names, native_node_names_);
      !common.empty()) {
    throw exceptions::TreeBuildError(
      "Found reserved node registration names in the node manifest. The following names are not allowed, because "
      "they refer to native behavior tree nodes: [ " +
      auto_apms_util::join(std::vector<std::string>(common.begin(), common.end()), ", ") + " ].");
  }

  for (const auto & [node_name, params] : tree_node_manifest.getInternalMap()) {
    // If the node is already registered
    if (registered_node_class_names_map_.find(node_name) != registered_node_class_names_map_.end()) {
      // Check whether the specified node class is different from the currently registered one by comparing the
      // respective plugin class names.
      if (registered_node_class_names_map_[node_name] == params.class_name) {
        // We assume that the manifest entry refers to the exact same node plugin, because all NodeRegistrationLoader
        // instances verify that there are no ambiguous class names during initialization. Since the node is already
        // registered, we may skip loading it as there's nothing new to do.
        RCLCPP_DEBUG(logger_, "Skipping already loaded Node '%s' (%s).", node_name.c_str(), params.class_name.c_str());
        continue;
      } else if (override) {
        // If it's actually a different class and override is true, register the new node plugin instead of the
        // current one.
        factory_.unregisterBuilder(node_name);
      } else {
        // If it's actually a different class and override is false, we must throw.
        throw exceptions::TreeBuildError(
          "The name '" + node_name + "' of a previously registered node (" +
          registered_node_class_names_map_[node_name] +
          ")' is also discovered parsing another node manifest, but with a different class name (" + params.class_name +
          "). You must explicitly set override=true to allow for overriding previously registered nodes.");
      }
    }

    // Check if the class we search for is actually available with the loader.
    if (!tree_node_loader_ptr_->isClassAvailable(params.class_name)) {
      if (all_node_classes_package_map_.find(params.class_name) == all_node_classes_package_map_.end()) {
        throw exceptions::TreeBuildError(
          "Node '" + node_name + " (" + params.class_name +
          ")' cannot be loaded, because the class name is not known to the class loader. "
          "Make sure that it's spelled correctly and registered by calling "
          "auto_apms_behavior_tree_declare_nodes() in the CMakeLists.txt of the "
          "corresponding package.");
      }
      throw exceptions::TreeBuildError(
        "Node '" + node_name + " (" + params.class_name +
        ")' cannot be loaded, because the corresponding resource belongs to excluded package '" +
        all_node_classes_package_map_.at(params.class_name) + "'.");
    }

    RCLCPP_DEBUG(
      logger_, "Loading behavior tree node '%s' (%s) from library %s.", node_name.c_str(), params.class_name.c_str(),
      tree_node_loader_ptr_->getClassLibraryPath(params.class_name).c_str());

    pluginlib::UniquePtr<NodeRegistrationInterface> plugin_instance;
    try {
      plugin_instance = tree_node_loader_ptr_->createUniqueInstance(params.class_name);
    } catch (const std::exception & e) {
      throw exceptions::TreeBuildError(
        "Failed to create an instance of node '" + node_name + " (" + params.class_name +
        ")'. Remember that the AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE "
        "macro must be called in the source file for the node class to be discoverable. "
        "Error message: " +
        e.what() + ".");
    }

    try {
      if (
        ros_node_wptr_.expired() || tree_node_waitables_callback_group_wptr_.expired() ||
        tree_node_waitables_executor_wptr_.expired()) {
        if (plugin_instance->requiresRosNodeContext()) {
          throw exceptions::TreeBuildError(
            "Constructor requires to pass an instance of RosNodeContext but the required pointers expired.");
        }
        plugin_instance->registerWithBehaviorTreeFactory(factory_, node_name);
      } else {
        RosNodeContext ros_node_context(
          ros_node_wptr_.lock(), tree_node_waitables_callback_group_wptr_.lock(),
          tree_node_waitables_executor_wptr_.lock(), params);
        plugin_instance->registerWithBehaviorTreeFactory(factory_, node_name, &ros_node_context);
      }

    } catch (const std::invalid_argument & e) {
      throw exceptions::TreeBuildError(
        "Cannot load node '" + node_name + " (" + params.class_name +
        ")' without a valid pointer to rclcpp::Node and rclcpp::CallbackGroup.");
    } catch (const std::exception & e) {
      throw exceptions::TreeBuildError(
        "Cannot load node '" + node_name + " (" + params.class_name + ")': " + e.what() + ".");
    }
    registered_node_class_names_map_[node_name] = params.class_name;
  }
  return *this;
}

std::unordered_map<std::string, BT::NodeType> TreeBuilder::getAvailableNodeTypeMap(bool include_native) const
{
  std::unordered_map<std::string, BT::NodeType> map;
  for (const auto & it : factory_.manifests()) {
    if (!include_native && native_node_names_.find(it.first) != native_node_names_.end()) continue;
    map.insert({it.first, it.second.type});
  };
  return map;
}

std::set<std::string> TreeBuilder::getAvailableNodeNames(bool include_native) const
{
  std::set<std::string> names;
  if (include_native) names = native_node_names_;
  for (const auto & it : registered_node_class_names_map_) names.insert(it.first);
  return names;
}

TreeBuilder & TreeBuilder::addNodeModelToDocument(bool include_native)
{
  tinyxml2::XMLDocument model_doc;
  getNodeModel(model_doc, include_native);  // Structure is already verified
  const tinyxml2::XMLElement * model_child =
    model_doc.RootElement()->FirstChildElement(TreeDocument::TREE_NODE_MODEL_ELEMENT_NAME);

  // Clone the memory of the node model element to the builder document
  tinyxml2::XMLNode * copied_child = model_child->DeepClone(&doc_);

  // Append the copied child to the root of the builder document
  doc_.RootElement()->InsertEndChild(copied_child);
  return *this;
}

bool TreeBuilder::verify() const
{
  try {
    BT::VerifyXML(doc_.str(), getAvailableNodeTypeMap(true));
  } catch (const BT::RuntimeError & e) {
    return false;
  }
  return true;
}

Tree TreeBuilder::instantiate(const std::string & root_tree_name, TreeBlackboardSharedPtr bb_ptr)
{
  Tree tree;
  try {
    factory_.registerBehaviorTreeFromText(doc_.str());
    tree = factory_.createTree(root_tree_name, bb_ptr);
    factory_.clearRegisteredBehaviorTrees();
  } catch (const std::exception & e) {
    throw exceptions::TreeBuildError("Error during instantiate(): " + std::string(e.what()));
  }
  return tree;
}

Tree TreeBuilder::instantiate(const TreeElement & tree, TreeBlackboardSharedPtr bb_ptr)
{
  return instantiate(tree.getName(), bb_ptr);
}

Tree TreeBuilder::instantiate(TreeBlackboardSharedPtr bb_ptr)
{
  if (doc_.hasRootTreeName()) return instantiate(doc_.getRootTreeName(), bb_ptr);
  throw exceptions::TreeBuildError(
    "Cannot instantiate tree without a root tree name. You must either specify the attribute '" +
    std::string(TreeDocument::ROOT_TREE_ATTRIBUTE_NAME) +
    "' of the tree document's root element or call setRootTreeName() to define the root tree. Alternatively, you may "
    "call a different signature of instantiate().");
}

void TreeBuilder::getNodeModel(tinyxml2::XMLDocument & doc, bool include_native) const
{
  // Load tree nodes model
  doc.Clear();
  if (doc.Parse(BT::writeTreeNodesModelXML(factory_, include_native).c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
    throw exceptions::TreeBuildError(
      "Error parsing the model of the currently loaded node plugins: " + std::string(doc.ErrorStr()));
  }
  const tinyxml2::XMLElement * root = doc.RootElement();
  if (!root) {
    throw exceptions::TreeBuildError("Node model document has no root element.");
  }
  const tinyxml2::XMLElement * model_ele =
    doc.RootElement()->FirstChildElement(TreeDocument::TREE_NODE_MODEL_ELEMENT_NAME);
  if (!model_ele) {
    throw exceptions::TreeBuildError(
      "Element <" + std::string(TreeDocument::TREE_NODE_MODEL_ELEMENT_NAME) +
      "> doesn't exist in node model document.");
  }
}

}  // namespace auto_apms_behavior_tree::core