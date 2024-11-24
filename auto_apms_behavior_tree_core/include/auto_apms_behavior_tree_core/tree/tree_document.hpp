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

#include <string>
#include <vector>

#include "auto_apms_behavior_tree_core/node/node_registration_params.hpp"

namespace auto_apms_behavior_tree::core
{

class TreeResource;
class TreeBuilder;

class TreeDocument : private tinyxml2::XMLDocument
{
  friend class TreeBuilder;

  using XMLElement = tinyxml2::XMLElement;

public:
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
  public:
    using PortValues = std::map<std::string, std::string>;

  protected:
    NodeElement(TreeBuilder * builder_ptr, XMLElement * ele_ptr);

  public:
    NodeElement insertNode(const std::string & name, const NodeElement * before_this = nullptr);

    NodeElement loadAndInsertNode(
      const std::string & node_name, const NodeRegistrationParams & registration_params,
      const NodeElement * before_this = nullptr);

    NodeElement insertSubTreeNode(const std::string & tree_name, const NodeElement * before_this = nullptr);

    NodeElement insertTree(const TreeElement & tree, const NodeElement * before_this = nullptr);

    NodeElement insertTreeFromDocument(
      TreeDocument & doc, const std::string & tree_name, const NodeElement * before_this = nullptr);

    NodeElement insertTreeFromDocument(TreeDocument & doc, const NodeElement * before_this = nullptr);

    NodeElement insertTreeFromString(
      const std::string & tree_str, const std::string & tree_name, const NodeElement * before_this = nullptr);

    NodeElement insertTreeFromString(const std::string & tree_str, const NodeElement * before_this = nullptr);

    NodeElement insertTreeFromFile(
      const std::string & path, const std::string & tree_name, const NodeElement * before_this = nullptr);

    NodeElement insertTreeFromFile(const std::string & path, const NodeElement * before_this = nullptr);

    NodeElement insertTreeFromResource(
      const TreeResource & resource, const std::string & tree_name, const NodeElement * before_this = nullptr);

    NodeElement insertTreeFromResource(const TreeResource & resource, const NodeElement * before_this = nullptr);

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

    NodeElement getFirstNode(const std::string & name = "") const;

    NodeElement & removeFirstChild(const std::string & name = "");

    NodeElement & removeChildren();

    bool hasChildren() const;

    std::string getRegistrationName() const;

    virtual std::string getName() const;

    std::string getFullyQualifiedName() const;

  private:
    NodeElement insertBeforeImpl(const NodeElement * before_this, XMLElement * add_this);

    NodeElement insertTreeImpl(const XMLElement * tree_root, const NodeElement * before_this = nullptr);

    static XMLElement * getFirstNodeImpl(XMLElement * ele, const std::string & name);

  protected:
    TreeBuilder * builder_ptr_;
    tinyxml2::XMLElement * ele_ptr_;
  };

  class TreeElement : public NodeElement
  {
    friend class TreeBuilder;

    using NodeElement::NodeElement;

  public:
    NodeElement & setPorts() = delete;

    std::string getName() const override;

    TreeElement & makeRoot();
  };

  TreeDocument();

  explicit TreeDocument(const TreeResource & resource);

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

private:
  XMLElement * getXMLElementForTreeWithName(const std::string & tree_name);
};

}  // namespace auto_apms_behavior_tree::core