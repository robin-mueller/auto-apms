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

#pragma once

#include <tinyxml2.h>

#include <functional>
#include <string>
#include <type_traits>
#include <vector>

#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include "auto_apms_behavior_tree_core/node/node_registration_loader.hpp"
#include "auto_apms_behavior_tree_core/node/node_registration_options.hpp"
#include "auto_apms_behavior_tree_core/tree/script.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/tree_node.h"
#include "rclcpp/rclcpp.hpp"

namespace auto_apms_behavior_tree::model
{
class SubTree;
}

namespace auto_apms_behavior_tree::core
{

class TreeResource;
class NodeModelType;

class TreeDocument : private tinyxml2::XMLDocument
{
  using XMLElement = tinyxml2::XMLElement;

  inline static const std::string LOGGER_NAME = "tree_document";

public:
  static inline const char BTCPP_FORMAT_ATTRIBUTE_NAME[] = "BTCPP_format";
  static inline const char BTCPP_FORMAT_DEFAULT_VERSION[] = "4";
  static inline const char ROOT_ELEMENT_NAME[] = "root";
  static inline const char ROOT_TREE_ATTRIBUTE_NAME[] = "main_tree_to_execute";
  static inline const char TREE_ELEMENT_NAME[] = "BehaviorTree";
  static inline const char SUBTREE_ELEMENT_NAME[] = "SubTree";
  static inline const char TREE_NAME_ATTRIBUTE_NAME[] = "ID";
  static inline const char TREE_NODE_MODEL_ELEMENT_NAME[] = "TreeNodesModel";
  static inline const char NODE_INSTANCE_NAME_ATTRIBUTE_NAME[] = "name";

  struct NodePortInfo
  {
    std::string port_name;
    std::string port_type;
    std::string port_default;
    std::string port_description;
    BT::PortDirection port_direction;
  };

  struct NodeModel
  {
    BT::NodeType type;
    std::vector<NodePortInfo> port_infos;
  };

  using NodeModelMap = std::map<std::string, NodeModel>;

  class TreeElement;

  class NodeElement
  {
    friend class TreeDocument;

  public:
    using PortValues = std::map<std::string, std::string>;
    using DeepApplyCallback = std::function<bool(NodeElement &)>;
    using ConstDeepApplyCallback = std::function<bool(const NodeElement &)>;

  protected:
    NodeElement(TreeDocument * doc_ptr, XMLElement * ele_ptr);

  public:
    NodeElement(const NodeElement & ele) = default;

    NodeElement & operator=(const NodeElement & other) = delete;

    NodeElement insertNode(const std::string & name, const NodeElement * before_this = nullptr);

    NodeElement insertNode(
      const std::string & node_name, const NodeRegistrationOptions & registration_params,
      const NodeElement * before_this = nullptr);

    template <class ModelT>
    typename std::enable_if_t<
      std::is_base_of_v<NodeModelType, ModelT> && !std::is_same_v<model::SubTree, ModelT>, ModelT>
    insertNode(const NodeElement * before_this = nullptr);

    template <class ModelT>
    typename std::enable_if_t<std::is_same_v<model::SubTree, ModelT>, model::SubTree> insertNode(
      const std::string & tree_name, const NodeElement * before_this = nullptr);

    model::SubTree insertSubTreeNode(const std::string & tree_name, const NodeElement * before_this = nullptr);

    NodeElement insertTree(const TreeElement & tree, const NodeElement * before_this = nullptr);

    NodeElement insertTreeFromDocument(
      const TreeDocument & other, const std::string & tree_name, const NodeElement * before_this = nullptr);

    NodeElement insertTreeFromDocument(const TreeDocument & other, const NodeElement * before_this = nullptr);

    NodeElement insertTreeFromString(
      const std::string & tree_str, const std::string & tree_name, const NodeElement * before_this = nullptr);

    NodeElement insertTreeFromString(const std::string & tree_str, const NodeElement * before_this = nullptr);

    NodeElement insertTreeFromFile(
      const std::string & path, const std::string & tree_name, const NodeElement * before_this = nullptr);

    NodeElement insertTreeFromFile(const std::string & path, const NodeElement * before_this = nullptr);

    NodeElement insertTreeFromResource(
      const TreeResource & resource, const std::string & tree_name, const NodeElement * before_this = nullptr);

    NodeElement insertTreeFromResource(const TreeResource & resource, const NodeElement * before_this = nullptr);

    bool hasChildren() const;

    NodeElement getFirstNode(const std::string & registration_name = "", const std::string & instance_name = "") const;

    template <class ModelT>
    typename std::enable_if_t<std::is_base_of_v<NodeModelType, ModelT>, ModelT> getFirstNode(
      const std::string & instance_name = "") const;

    NodeElement & removeFirstChild(const std::string & registration_name = "", const std::string & instance_name = "");

    template <class ModelT>
    typename std::enable_if_t<std::is_base_of_v<NodeModelType, ModelT>, NodeElement &> removeFirstChild(
      const std::string & instance_name = "");

    NodeElement & removeChildren();

    const std::vector<std::string> & getPortNames() const;

    PortValues getPorts() const;

    /**
     * @brief Set the node's ports.
     *
     * This method also verifies that @p port_values complies with the node's port model
     * and throws an exception if any values for unkown port names are provided.
     *
     * @param port_values Port values to be used to fill the corresponding attributes of the node element.
     * @return Reference to the modified instance.
     * @throws exceptions::TreeBuildError if @p port_values contains unkown keys, that is names for ports that are not
     * implemented.
     */
    NodeElement & setPorts(const PortValues & port_values);

    NodeElement & resetPorts();

    NodeElement & setConditionalScript(BT::PreCond type, const Script & script);

    NodeElement & setConditionalScript(BT::PostCond type, const Script & script);

    NodeElement & setName(const std::string & instance_name);

    /// @brief Name of the behavior tree node given during registration.
    virtual std::string getRegistrationName() const;

    /// @brief Name of the behavior tree node given to this specific instance by the developer.
    virtual std::string getName() const;

    std::string getFullyQualifiedName() const;

    const std::vector<NodeElement> deepApply(ConstDeepApplyCallback apply_callback) const;

    std::vector<NodeElement> deepApply(DeepApplyCallback apply_callback);

  private:
    NodeElement insertBeforeImpl(const NodeElement * before_this, XMLElement * add_this);

    static void deepApplyImpl(
      const NodeElement & parent, ConstDeepApplyCallback apply_callback, std::vector<NodeElement> & vec);

    static void deepApplyImpl(NodeElement & parent, DeepApplyCallback apply_callback, std::vector<NodeElement> & vec);

  protected:
    TreeDocument * doc_ptr_;
    tinyxml2::XMLElement * ele_ptr_;

  private:
    std::vector<std::string> port_names_;
    PortValues port_default_values_;
  };

  class TreeElement : public NodeElement
  {
    friend class TreeDocument;

  protected:
    TreeElement(TreeDocument * doc_ptr, XMLElement * ele_ptr);

  public:
    TreeElement(const TreeElement & ele) = default;

    TreeElement & operator=(const TreeElement & other);

    std::string getName() const override;

    TreeElement & makeRoot();

    NodeManifest getRequiredNodeManifest() const;

    BT::Result verify() const;

    std::string writeToString() const;

    TreeElement & removeFirstChild(const std::string & registration_name = "", const std::string & instance_name = "");

    template <class ModelT>
    typename std::enable_if_t<std::is_base_of_v<NodeModelType, ModelT>, TreeElement &> removeFirstChild(
      const std::string & instance_name = "");

    TreeElement & removeChildren();

    /* Not supported methods for TreeElement instances */

    const std::vector<std::string> & getPortNames() = delete;
    PortValues getPorts() = delete;
    NodeElement & setPorts() = delete;
    NodeElement & resetPorts() = delete;
    NodeElement & setConditionalScript() = delete;
    NodeElement & setName() = delete;
  };

  TreeDocument(
    const std::string & format_version = BTCPP_FORMAT_DEFAULT_VERSION,
    NodeRegistrationLoader::SharedPtr tree_node_loader = NodeRegistrationLoader::make_shared());

  virtual ~TreeDocument() = default;

  TreeDocument & mergeTreeDocument(const XMLDocument & other, bool adopt_root_tree = false);

  TreeDocument & mergeTreeDocument(const TreeDocument & other, bool adopt_root_tree = false);

  TreeDocument & mergeString(const std::string & tree_str, bool adopt_root_tree = false);

  TreeDocument & mergeFile(const std::string & path, bool adopt_root_tree = false);

  TreeDocument & mergeResource(const TreeResource & resource, bool adopt_root_tree = false);

  TreeDocument & mergeTree(const TreeElement & tree, bool make_root_tree = false);

  TreeElement newTree(const std::string & tree_name);

  TreeElement newTree(const TreeElement & other_tree);

  TreeElement newTreeFromDocument(const TreeDocument & other, const std::string & tree_name = "");

  TreeElement newTreeFromString(const std::string & tree_str, const std::string & tree_name = "");

  TreeElement newTreeFromFile(const std::string & path, const std::string & tree_name = "");

  TreeElement newTreeFromResource(const TreeResource & resource, const std::string & tree_name = "");

  bool hasTree(const std::string & tree_name) const;

  TreeElement getTree(const std::string & tree_name);

  TreeDocument & setRootTreeName(const std::string & tree_name);

  bool hasRootTreeName() const;

  std::string getRootTreeName() const;

  TreeElement getRootTree();

  TreeDocument & removeTree(const std::string & tree_name);

  TreeDocument & removeTree(const TreeElement & tree);

  std::vector<std::string> getAllTreeNames() const;

  /**
   * @brief Load behavior tree node plugins and register them with the internal behavior tree factory.
   *
   * This makes it possible to add any nodes specified in @p tree_node_manifest to the tree.
   *
   * @param[in] tree_node_manifest Parameters for locating and configuring the behavior tree node plugins.
   * @param[in] override If @p tree_node_manifest specifies nodes that have already been registered, unregister the
   * existing plugin and use the new one instead.
   * @throw exceptions::NodeRegistrationError if registration fails.
   */
  virtual TreeDocument & registerNodes(const NodeManifest & tree_node_manifest, bool override = false);

  std::set<std::string> getAvailableNodeNames(bool include_native = true) const;

  NodeManifest getRequiredNodeManifest() const;

  TreeDocument & addNodeModel(bool include_native = false);

  static NodeModelMap getNodeModel(tinyxml2::XMLDocument & doc);

  NodeModelMap getNodeModel(bool include_native = false) const;

  // Verify the structure of this tree document and that all nodes are registered with the factory
  BT::Result verify() const;

  std::string writeToString() const;

  void writeToFile(const std::string & path) const;

  TreeDocument & reset();

private:
  template <typename ReturnT, typename DocumentT>
  static ReturnT getXMLElementForTreeWithNameImpl(DocumentT & doc, const std::string & tree_name);

  const XMLElement * getXMLElementForTreeWithName(const std::string & tree_name) const;

  XMLElement * getXMLElementForTreeWithName(const std::string & tree_name);

  const std::map<std::string, std::string> all_node_classes_package_map_;
  const std::set<std::string> native_node_names_;
  std::string format_version_;
  NodeRegistrationLoader::SharedPtr tree_node_loader_ptr_;
  NodeManifest registered_nodes_manifest_;

protected:
  BT::BehaviorTreeFactory factory_;
  rclcpp::Logger logger_;
  rclcpp::Node::WeakPtr ros_node_wptr_;
  rclcpp::CallbackGroup::WeakPtr tree_node_waitables_callback_group_wptr_;
  rclcpp::executors::SingleThreadedExecutor::WeakPtr tree_node_waitables_executor_wptr_;
  bool only_non_ros_nodes_ = false;
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

template <typename T, typename = void>
struct has_static_method_registrationOptions : std::false_type
{
};

template <typename T>
struct has_static_method_registrationOptions<
  T, typename std::enable_if_t<std::is_same_v<decltype(T::registrationOptions()), NodeRegistrationOptions>>>
: std::true_type
{
};

template <class ModelT>
inline
  typename std::enable_if_t<std::is_base_of_v<NodeModelType, ModelT> && !std::is_same_v<model::SubTree, ModelT>, ModelT>
  TreeDocument::NodeElement::insertNode(const NodeElement * before_this)
{
  std::unique_ptr<NodeElement> temp;
  // See if model implements static method registrationOptions() (Models for native nodes don't)
  if constexpr (has_static_method_registrationOptions<ModelT>::value) {
    temp = std::make_unique<NodeElement>(insertNode(ModelT::name(), ModelT::registrationOptions(), before_this));
  } else {
    temp = std::make_unique<NodeElement>(insertNode(ModelT::name(), before_this));
  }
  return ModelT(temp->doc_ptr_, temp->ele_ptr_);
}

template <class ModelT>
inline typename std::enable_if_t<std::is_same_v<model::SubTree, ModelT>, model::SubTree>
TreeDocument::NodeElement::insertNode(const std::string & tree_name, const NodeElement * before_this)
{
  return insertSubTreeNode(tree_name, before_this);
}

template <class ModelT>
inline typename std::enable_if_t<std::is_base_of_v<NodeModelType, ModelT>, ModelT>
core::TreeDocument::NodeElement::getFirstNode(const std::string & instance_name) const
{
  const NodeElement ele = getFirstNode(ModelT::name(), instance_name);
  return ModelT(ele.doc_ptr_, ele.ele_ptr_);
}

template <class ModelT>
inline typename std::enable_if_t<std::is_base_of_v<NodeModelType, ModelT>, TreeDocument::NodeElement &>
core::TreeDocument::NodeElement::removeFirstChild(const std::string & instance_name)
{
  return removeFirstChild(ModelT::name(), instance_name);
}

template <class ModelT>
inline typename std::enable_if_t<std::is_base_of_v<NodeModelType, ModelT>, TreeDocument::TreeElement &>
TreeDocument::TreeElement::removeFirstChild(const std::string & instance_name)
{
  NodeElement::removeFirstChild<ModelT>(instance_name);
  return *this;
}

}  // namespace auto_apms_behavior_tree::core