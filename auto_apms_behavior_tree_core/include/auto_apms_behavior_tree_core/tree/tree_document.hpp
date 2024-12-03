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

#include "auto_apms_behavior_tree_core/node/node_model_type.hpp"
#include "auto_apms_behavior_tree_core/node/node_registration_options.hpp"
#include "auto_apms_behavior_tree_core/tree/script.hpp"
#include "behaviortree_cpp/tree_node.h"

namespace auto_apms_behavior_tree::core
{

class TreeResource;
class TreeBuilder;

class TreeDocument : private tinyxml2::XMLDocument
{
  friend class TreeBuilder;

  using XMLElement = tinyxml2::XMLElement;

public:
  using NodePreCondition = BT::PreCond;
  using NodePostCondition = BT::PostCond;

  static inline const char BTCPP_FORMAT_ATTRIBUTE_NAME[] = "BTCPP_format";
  static inline const char BTCPP_FORMAT_DEFAULT_VERSION[] = "4";
  static inline const char ROOT_ELEMENT_NAME[] = "root";
  static inline const char ROOT_TREE_ATTRIBUTE_NAME[] = "main_tree_to_execute";
  static inline const char TREE_ELEMENT_NAME[] = "BehaviorTree";
  static inline const char SUBTREE_ELEMENT_NAME[] = "SubTree";
  static inline const char TREE_NAME_ATTRIBUTE_NAME[] = "ID";
  static inline const char TREE_NODE_MODEL_ELEMENT_NAME[] = "TreeNodesModel";
  static inline const char NODE_INSTANCE_NAME_ATTRIBUTE_NAME[] = "name";

  class TreeElement;

  class NodeElement
  {
    friend class TreeBuilder;

  public:
    using PortValues = NodeModelType::PortValues;
    using DeepApplyCallback = std::function<bool(NodeElement &)>;
    using ConstDeepApplyCallback = std::function<bool(const NodeElement &)>;

  protected:
    NodeElement(TreeBuilder * builder_ptr, XMLElement * ele_ptr);

  public:
    NodeElement insertNode(const std::string & name, const NodeElement * before_this = nullptr);

    NodeElement insertNode(
      const std::string & node_name, const NodeRegistrationOptions & registration_params,
      const NodeElement * before_this = nullptr);

    NodeElement insertNode(const NodeModelType & model, const NodeElement * before_this = nullptr);

    template <class ModelT>
    typename std::enable_if_t<std::is_base_of_v<NodeModelType, ModelT>, NodeElement> insertNode(
      const NodeElement * before_this = nullptr);

    NodeElement insertSubTreeNode(const std::string & tree_name, const NodeElement * before_this = nullptr);

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

    NodeElement getFirstNode(const std::string & name = "") const;

    template <class ModelT>
    typename std::enable_if_t<std::is_base_of_v<NodeModelType, ModelT>, NodeElement> getFirstNode() const;

    NodeElement & removeFirstChild(const std::string & name = "");

    template <class ModelT>
    typename std::enable_if_t<std::is_base_of_v<NodeModelType, ModelT>, NodeElement &> removeFirstChild();

    NodeElement & removeChildren();

    /**
     * @brief Set the node's ports.
     *
     * Will use each port's default value if one is implemented and the corresponding key doesn't exist in @p
     * port_values. If the argument is omitted, only default values will be used.
     *
     * If @p verify is `true`, the method additionally verifies that @p port_values complies with the node's port model
     * and throws an exception if there are any values for unkown port names. If set to `false`, no error is raised in
     * such a case and values for unkown port names are silently discarded thus won't appear in the element's list of
     * port attributes.
     *
     * @param port_values Port values to be used to fill the corresponding attributes of the node element.
     * @param verify Flag wether to verify that @p port_values complies with the node's port model.
     * @return Reference to the modified TreeBuilder object.
     * @throws exceptions::TreeBuildError if @p verify is set to `true` and @p port_values contains keys that are not
     * associated with the node.
     */
    NodeElement & setPorts(const PortValues & port_values = {}, bool verify = true);

    NodeElement & setPreCondition(NodePreCondition type, const Script & script);

    NodeElement & setPostCondition(NodePostCondition type, const Script & script);

    std::string getRegistrationName() const;

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
    TreeBuilder * builder_ptr_;
    tinyxml2::XMLElement * ele_ptr_;

  private:
    std::vector<std::string> port_keys_;
    std::map<std::string, std::string> port_default_values_;
  };

  class TreeElement : public NodeElement
  {
    friend class TreeBuilder;

  protected:
    TreeElement(TreeBuilder * builder_ptr, XMLElement * ele_ptr);

  public:
    std::string getName() const override;

    TreeElement & makeRoot();

    /* Not supported methods for a TreeElement instances */

    NodeElement & setPorts() = delete;
    NodeElement & setPreCondition() = delete;
    NodeElement & setPostCondition() = delete;
  };

  TreeDocument(const std::string & format_version = BTCPP_FORMAT_DEFAULT_VERSION);

  TreeDocument(const TreeDocument & other);

  TreeDocument & operator=(const TreeDocument & other);

  TreeDocument & merge(const XMLDocument & other, bool adopt_root_tree = false);

  TreeDocument & mergeString(const std::string & tree_str, bool adopt_root_tree = false);

  TreeDocument & mergeFile(const std::string & path, bool adopt_root_tree = false);

  TreeDocument & mergeResource(const TreeResource & resource, bool adopt_root_tree = false);

  bool isExistingTreeName(const std::string & tree_name) const;

  TreeDocument & setRootTreeName(const std::string & tree_name);

  bool hasRootTreeName() const;

  std::string getRootTreeName() const;

  TreeDocument & removeTreeWithName(const std::string & tree_name);

  std::vector<std::string> getAllTreeNames() const;

  std::string str() const;

  TreeDocument & reset();

  TreeDocument & reset(const TreeDocument & new_doc);

protected:
  const XMLElement * getXMLElementForTreeWithName(const std::string & tree_name) const;

  XMLElement * getXMLElementForTreeWithName(const std::string & tree_name);

private:
  template <typename ReturnT, typename DocumentT>
  static ReturnT getXMLElementForTreeWithNameImpl(DocumentT & doc, const std::string & tree_name);

  const std::string format_version_;
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

template <class ModelT>
inline typename std::enable_if_t<std::is_base_of_v<NodeModelType, ModelT>, TreeDocument::NodeElement>
TreeDocument::NodeElement::insertNode(const NodeElement * before_this)
{
  return insertNode(ModelT(), before_this);
}

template <class ModelT>
inline typename std::enable_if_t<std::is_base_of_v<NodeModelType, ModelT>, TreeDocument::NodeElement>
TreeDocument::NodeElement::getFirstNode() const
{
  return getFirstNode(ModelT().getRegistrationName());
}

template <class ModelT>
inline typename std::enable_if_t<std::is_base_of_v<NodeModelType, ModelT>, TreeDocument::NodeElement &>
TreeDocument::NodeElement::removeFirstChild()
{
  return removeFirstChild(ModelT().getRegistrationName());
}

}  // namespace auto_apms_behavior_tree::core