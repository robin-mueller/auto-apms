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

#include <map>

#include "auto_apms_behavior_tree_core/node/node_registration_options.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "behaviortree_cpp/basic_types.h"

#define AUTO_APMS_BEHAVIOR_TREE_CORE_DEFINE_NON_LEAF_THISREF_METHODS(ClassType)                                    \
  ClassType & removeFirstChild(const std::string & registration_name = "", const std::string & instance_name = "") \
  {                                                                                                                \
    NodeElement::removeFirstChild(registration_name, instance_name);                                               \
    return *this;                                                                                                  \
  }                                                                                                                \
  template <class ModelT>                                                                                          \
  typename std::enable_if_t<std::is_base_of_v<NodeModelType, ModelT>, ClassType &> removeFirstChild(               \
    const std::string & instance_name = "")                                                                        \
  {                                                                                                                \
    NodeElement::removeFirstChild<ModelT>(instance_name);                                                          \
    return *this;                                                                                                  \
  }                                                                                                                \
  ClassType & removeChildren()                                                                                     \
  {                                                                                                                \
    NodeElement::removeChildren();                                                                                 \
    return *this;                                                                                                  \
  }

#define AUTO_APMS_BEHAVIOR_TREE_CORE_DEFINE_LEAF_THISREF_METHODS(ClassType)                                      \
  ClassType & setPorts(const auto_apms_behavior_tree::core::TreeDocument::NodeElement::PortValues & port_values) \
  {                                                                                                              \
    NodeElement::setPorts(port_values);                                                                          \
    return *this;                                                                                                \
  }                                                                                                              \
  ClassType & resetPorts()                                                                                       \
  {                                                                                                              \
    NodeElement::resetPorts();                                                                                   \
    return *this;                                                                                                \
  }                                                                                                              \
  ClassType & setConditionalScript(BT::PreCond type, const auto_apms_behavior_tree::core::Script & script)       \
  {                                                                                                              \
    NodeElement::setConditionalScript(type, script);                                                             \
    return *this;                                                                                                \
  }                                                                                                              \
  ClassType & setConditionalScript(BT::PostCond type, const auto_apms_behavior_tree::core::Script & script)      \
  {                                                                                                              \
    NodeElement::setConditionalScript(type, script);                                                             \
    return *this;                                                                                                \
  }                                                                                                              \
  ClassType & setName(const std::string & instance_name)                                                         \
  {                                                                                                              \
    NodeElement::setName(instance_name);                                                                         \
    return *this;                                                                                                \
  }

namespace auto_apms_behavior_tree
{
namespace core
{

class TreeBuilder;

class NodeModelType : public TreeDocument::NodeElement
{
protected:
  using NodeElement::NodeElement;

public:
  using RegistrationOptions = NodeRegistrationOptions;
  using PortInfos = std::map<std::string, BT::PortInfo>;

  /// @copydoc TreeDocument::NodeElement::getRegistrationName()
  virtual std::string getRegistrationName() const override = 0;
};

class LeafNodeModelType : public NodeModelType
{
protected:
  using NodeModelType::NodeModelType;

public:
  /* Not supported methods for LeafNodeModelType instances */

  LeafNodeModelType insertNode() = delete;
  LeafNodeModelType insertSubTreeNode() = delete;
  LeafNodeModelType insertTree() = delete;
  LeafNodeModelType insertTreeFromDocument() = delete;
  LeafNodeModelType insertTreeFromString() = delete;
  LeafNodeModelType insertTreeFromFile() = delete;
  LeafNodeModelType insertTreeFromResource() = delete;
  LeafNodeModelType & removeFirstChild() = delete;
  LeafNodeModelType & removeChildren() = delete;
};

}  // namespace core

namespace model
{

class SubTree : public core::LeafNodeModelType
{
  friend class core::TreeDocument::NodeElement;

private:
  using LeafNodeModelType::LeafNodeModelType;

public:
  /// @copydoc TreeDocument::NodeElement::getRegistrationName()
  static std::string name();

  /// @brief Type of the behavior tree node.
  static BT::NodeType type();

  /// @copydoc TreeDocument::NodeElement::getRegistrationName()
  std::string getRegistrationName() const override final;

  AUTO_APMS_BEHAVIOR_TREE_CORE_DEFINE_LEAF_THISREF_METHODS(SubTree)

  SubTree & setBlackboardRemapping(const PortValues & remapping);

  /**
   * @brief Set automatic blackboard remapping.
   * @param val `true` to enable and `false` to disable.
   */
  SubTree & set_auto_remap(bool val = false);

  /**
   * @brief Get automatic blackboard remapping.
   * @return Boolean flag for the currently configured option.
   */
  bool get_auto_remap() const;
};

}  // namespace model
}  // namespace auto_apms_behavior_tree