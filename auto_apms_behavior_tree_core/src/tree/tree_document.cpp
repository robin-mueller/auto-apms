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

#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"

#include "auto_apms_behavior_tree_core/builder.hpp"
#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_resource.hpp"
#include "auto_apms_util/container.hpp"
#include "auto_apms_util/string.hpp"

namespace auto_apms_behavior_tree::core
{

std::vector<std::string> getAllTreeNamesImpl(const tinyxml2::XMLDocument & doc)
{
  std::vector<std::string> names;
  if (const tinyxml2::XMLElement * root = doc.RootElement()) {
    if (strcmp(root->Name(), TreeDocument::ROOT_ELEMENT_NAME) == 0) {
      // Found root element
      for (const tinyxml2::XMLElement * child = root->FirstChildElement(); child != nullptr;
           child = child->NextSiblingElement()) {
        if (strcmp(TreeDocument::TREE_ELEMENT_NAME, child->Name()) == 0) {
          if (const char * name = child->Attribute(TreeDocument::TREE_NAME_ATTRIBUTE_NAME)) {
            names.push_back(name);
          } else {
            throw exceptions::TreeDocumentError(
              "Cannot get tree name, because required attribute '" +
              std::string(TreeDocument::TREE_NAME_ATTRIBUTE_NAME) + "' is missing.");
          }
        }
      }
    } else if (strcmp(root->Name(), TreeDocument::TREE_ELEMENT_NAME) == 0) {
      // Found behavior tree element as root
      if (const char * name = root->Attribute(TreeDocument::TREE_NAME_ATTRIBUTE_NAME)) names.push_back(name);
    }
  }
  return names;
}

TreeDocument::NodeElement::NodeElement(TreeBuilder * builder_ptr, XMLElement * ele_ptr)
: builder_ptr_(builder_ptr), ele_ptr_(ele_ptr)
{
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertNode(
  const std::string & node_name, const NodeElement * before_this)
{
  if (const std::set<std::string> names = builder_ptr_->getAvailableNodeNames(true);
      names.find(node_name) == names.end()) {
    throw exceptions::TreeDocumentError(
      "Cannot insert unkown node '" + node_name +
      "'. The TreeBuilder must have registered the node before you may insert it using insertNode(). To prevent this "
      "error, make sure loadNodePlugins() is being called with a corresponding node manifest or provide the "
      "additional argument registration_params of insertNode().");
  }
  XMLElement * ele = builder_ptr_->doc_.NewElement(node_name.c_str());
  return insertBeforeImpl(before_this, ele).setPorts();  // Additionally add default port values
}

TreeDocument::NodeElement TreeDocument::NodeElement::loadAndInsertNode(
  const std::string & node_name, const NodeRegistrationParams & registration_params, const NodeElement * before_this)
{
  if (const std::set<std::string> names = builder_ptr_->getAvailableNodeNames(false);
      names.find(node_name) == names.end()) {
    // Try to load the node if it isn't registered yet
    builder_ptr_->loadNodePlugins(NodeManifest({{node_name, registration_params}}));
  }
  return insertNode(node_name, before_this);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertSubTreeNode(
  const std::string & tree_name, const NodeElement * before_this)
{
  return insertNode(SUBTREE_ELEMENT_NAME, before_this).setPorts({{TREE_NAME_ATTRIBUTE_NAME, tree_name}}, false);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTree(
  const TreeElement & tree, const NodeElement * before_this)
{
  return insertTreeImpl(tree.ele_ptr_, before_this);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromDocument(
  TreeDocument & doc, const std::string & tree_name, const NodeElement * before_this)
{
  const XMLElement * ele = doc.getXMLElementForTreeWithName(tree_name);
  if (doc.getAllTreeNames().size() > 1) {
    // Merge additional trees into this document to support potential subtree nodes within other
    builder_ptr_->doc_.merge(doc.removeTreeWithName(tree_name));
  }
  return insertTreeImpl(ele, before_this);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromDocument(
  TreeDocument & doc, const NodeElement * before_this)
{
  return insertTreeFromDocument(doc, doc.getRootTreeName());
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromString(
  const std::string & tree_str, const std::string & tree_name, const NodeElement * before_this)
{
  TreeDocument insert_doc;
  insert_doc.mergeString(tree_str);
  return insertTreeFromDocument(insert_doc, tree_name);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromString(
  const std::string & tree_str, const NodeElement * before_this)
{
  TreeDocument insert_doc;
  insert_doc.mergeString(tree_str);
  return insertTreeFromDocument(insert_doc);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromFile(
  const std::string & path, const std::string & tree_name, const NodeElement * before_this)
{
  TreeDocument insert_doc;
  insert_doc.mergeFile(path);
  return insertTreeFromDocument(insert_doc, tree_name);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromFile(
  const std::string & path, const NodeElement * before_this)
{
  TreeDocument insert_doc;
  insert_doc.mergeFile(path);
  return insertTreeFromDocument(insert_doc);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromResource(
  const TreeResource & resource, const std::string & tree_name, const NodeElement * before_this)
{
  TreeDocument insert_doc;
  insert_doc.mergeFile(resource.tree_file_path_);
  return insertTreeFromDocument(insert_doc, tree_name);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromResource(
  const TreeResource & resource, const NodeElement * before_this)
{
  return insertTreeFromResource(resource, resource.getRootTreeName(), before_this);
}

TreeDocument::NodeElement & TreeDocument::NodeElement::setPorts(const PortValues & port_values, bool verify)
{
  const char * node_name = ele_ptr_->Name();
  std::vector<std::string> implemented_ports_keys;
  if (strcmp(node_name, SUBTREE_ELEMENT_NAME) == 0) {
    if (port_values.find(TREE_NAME_ATTRIBUTE_NAME) == port_values.end()) {
      throw exceptions::TreeDocumentError(
        "Cannot insert <" + std::string(SUBTREE_ELEMENT_NAME) + "> node, because required key '" +
        TREE_NAME_ATTRIBUTE_NAME + "' for identifying the associated tree is missing in port_values.");
    }
    if (const std::string & tree_name = port_values.at(TREE_NAME_ATTRIBUTE_NAME);
        !builder_ptr_->doc_.isExistingTreeName(tree_name)) {
      throw exceptions::TreeDocumentError(
        "Cannot insert <" + std::string(SUBTREE_ELEMENT_NAME) + "> node, because associated tree '" + tree_name +
        "' doesn't exist.");
    }

    // Tree name attribute is not implemented as a port, so we must add it to the vector manually.
    implemented_ports_keys.push_back(TREE_NAME_ATTRIBUTE_NAME);
  }

  // Extract the port names and default values from the model.
  // Note: The basic structure of the document has already been verified.
  tinyxml2::XMLDocument model_doc;
  builder_ptr_->getNodeModel(model_doc, true);
  const XMLElement * node_ele =
    model_doc.RootElement()->FirstChildElement(TREE_NODE_MODEL_ELEMENT_NAME)->FirstChildElement();
  while (node_ele && !node_ele->Attribute("ID", node_name)) {
    node_ele = node_ele->NextSiblingElement();
  }
  if (!node_ele) {
    throw exceptions::TreeDocumentError(
      "Cannot set ports to node '" + std::string(node_name) + "' because it cannot be found in tree nodes model.");
  }
  PortValues default_values;
  for (const XMLElement * port_ele = node_ele->FirstChildElement(); port_ele != nullptr;
       port_ele = port_ele->NextSiblingElement()) {
    if (std::regex_match(port_ele->Name(), std::regex("input_port|output_port|inout_port"))) {
      const char * port_key = port_ele->Attribute("name");
      const char * port_default = port_ele->Attribute("default");
      implemented_ports_keys.push_back(port_key);
      if (port_key && port_default) {
        default_values[port_key] = port_default;
      }
    }
  }

  // Verify port_values
  if (verify) {
    std::vector<std::string> unkown_keys;
    for (const auto & [key, val] : port_values) {
      if (!auto_apms_util::contains(implemented_ports_keys, key)) unkown_keys.push_back(key);
    }
    if (!unkown_keys.empty()) {
      throw exceptions::TreeDocumentError(
        "Verification of argument port_values failed. The following port keys are not implemented by node '" +
        std::string(node_name) + "': [" + rcpputils::join(unkown_keys, ", ") + "].");
    }
  }

  // Clear attributes
  const tinyxml2::XMLAttribute * attribute = ele_ptr_->FirstAttribute();
  while (attribute) {
    const char * attr_name = attribute->Name();
    attribute = attribute->Next();
    ele_ptr_->DeleteAttribute(attr_name);
  }

  // Populate the node's attributes
  for (const auto & key : implemented_ports_keys) {
    if (port_values.find(key) != port_values.end()) {
      ele_ptr_->SetAttribute(key.c_str(), port_values.at(key).c_str());
    } else if (default_values.find(key) != default_values.end()) {
      ele_ptr_->SetAttribute(key.c_str(), default_values.at(key).c_str());
    }
    // If port is neither specified in port_values nor has it a default value, ignore.
  }
  return *this;
}

TreeDocument::NodeElement TreeDocument::NodeElement::getFirstNode(const std::string & name) const
{
  if (name.empty()) return NodeElement(builder_ptr_, ele_ptr_->FirstChildElement());
  if (XMLElement * ele = getFirstNodeImpl(ele_ptr_->FirstChildElement(), name)) return NodeElement(builder_ptr_, ele);
  throw exceptions::TreeDocumentError(
    "Cannot find node '" + name + "' in parent element '" + getRegistrationName() + "'.");
}

TreeDocument::NodeElement & TreeDocument::NodeElement::removeFirstChild(const std::string & name)
{
  ele_ptr_->DeleteChild(getFirstNode(name).ele_ptr_);
  return *this;
}

TreeDocument::NodeElement & TreeDocument::NodeElement::removeChildren()
{
  ele_ptr_->DeleteChildren();
  return *this;
}

bool TreeDocument::NodeElement::hasChildren() const { return ele_ptr_->FirstChild() == nullptr ? false : true; }

std::string TreeDocument::NodeElement::getRegistrationName() const { return ele_ptr_->Name(); }

std::string TreeDocument::NodeElement::getName() const
{
  if (const char * name = ele_ptr_->Attribute(NODE_INSTANCE_NAME_ATTRIBUTE_NAME)) return name;
  return getRegistrationName();
}

std::string TreeDocument::NodeElement::getFullyQualifiedName() const
{
  std::string registration_name = getRegistrationName();
  std::string instance_name = getName();
  if (registration_name == instance_name) return registration_name;
  return instance_name + " (" + registration_name + ")";
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertBeforeImpl(
  const NodeElement * before_this, XMLElement * add_this)
{
  if (before_this) {
    XMLElement * prev = nullptr;
    XMLElement * curr = ele_ptr_->FirstChildElement();

    // Traverse through siblings of first child to find the before_this node.
    // If there are no siblings, add the first child to this;
    bool found = false;
    while (curr) {
      if (curr == before_this->ele_ptr_) {
        found = true;
        break;
      }
      prev = curr;
      curr = curr->NextSiblingElement();
    }
    if (prev) {
      if (found) {
        ele_ptr_->InsertAfterChild(prev, add_this);
      } else {
        throw exceptions::TreeDocumentError(
          "NodeElement before_this (" + before_this->getFullyQualifiedName() + ") is not a child of " +
          getFullyQualifiedName() + ".");
      }
    } else {
      ele_ptr_->InsertFirstChild(add_this);
    }
  } else {
    ele_ptr_->InsertEndChild(add_this);
  }
  return NodeElement(builder_ptr_, add_this);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeImpl(
  const XMLElement * tree_root, const NodeElement * before_this)
{
  const XMLElement * child = tree_root->FirstChildElement();
  if (!child) {
    throw exceptions::TreeDocumentError("Cannot insert tree without children.");
  }
  // Make sure the memory is managed by this document
  XMLElement * copied_child = child->DeepClone(&builder_ptr_->doc_)->ToElement();
  return insertBeforeImpl(before_this, copied_child);
}

TreeDocument::XMLElement * TreeDocument::NodeElement::getFirstNodeImpl(XMLElement * ele, const std::string & name)
{
  if (!ele) return nullptr;

  // Match with the first node that is registered under the given name
  if (NodeElement(nullptr, ele).getRegistrationName() == name) return ele;

  // Also match with instance name if given
  if (NodeElement(nullptr, ele).getName() == name) return ele;

  // Recursively search in the child elements
  if (XMLElement * child = getFirstNodeImpl(ele->FirstChildElement(), name)) return child;

  // Search in the siblings if not found in children
  return getFirstNodeImpl(ele->NextSiblingElement(), name);
}

std::string TreeDocument::TreeElement::getName() const
{
  if (const char * name = ele_ptr_->Attribute(TREE_NAME_ATTRIBUTE_NAME)) return name;
  throw exceptions::TreeDocumentError(
    "<" + std::string(TREE_ELEMENT_NAME) + "> elements must have an attribute '" + TREE_NAME_ATTRIBUTE_NAME + "'.");
}

TreeDocument::TreeElement & TreeDocument::TreeElement::makeRoot()
{
  builder_ptr_->setRootTreeName(*this);
  return *this;
}

void setUpTreeDocument(tinyxml2::XMLDocument & doc)
{
  tinyxml2::XMLElement * root_ele = doc.NewElement(TreeDocument::ROOT_ELEMENT_NAME);
  root_ele->SetAttribute("BTCPP_format", "4");
  doc.InsertFirstChild(root_ele);
}

TreeDocument::TreeDocument() : XMLDocument() { setUpTreeDocument(*this); }

TreeDocument::TreeDocument(const TreeResource & resource) : TreeDocument() { mergeResource(resource, true); }

TreeDocument & TreeDocument::merge(const XMLDocument & other, bool adopt_root_tree)
{
  const XMLElement * other_root = other.RootElement();
  if (!other_root) {
    throw exceptions::TreeDocumentError("Cannot merge tree documents: other_root is nullptr.");
  };

  const std::vector<std::string> other_tree_names = getAllTreeNamesImpl(other);

  // Verify that there are no duplicate tree names
  const auto common_tree_names = auto_apms_util::getCommonElements(getAllTreeNames(), other_tree_names);
  if (!common_tree_names.empty()) {
    throw exceptions::TreeDocumentError(
      "Cannot merge trees: The following trees are already defined: [ " + rcpputils::join(common_tree_names, ", ") +
      " ].");
  }

  if (strcmp(other_root->Name(), ROOT_ELEMENT_NAME) == 0) {
    // Iterate over all the children of other root.
    const XMLElement * child = other_root->FirstChildElement();  // Allow include tags
    if (!child) {
      throw exceptions::TreeDocumentError(
        "Cannot merge tree documents: Root element of other document has no children.");
    }
    for (; child != nullptr; child = child->NextSiblingElement()) {
      if (strcmp(child->Name(), TREE_NODE_MODEL_ELEMENT_NAME) == 0) {
        // Disregard tree node model elements
        continue;
      }
      RootElement()->InsertEndChild(child->DeepClone(this));
    }

    if (adopt_root_tree) {
      // After (!) all tree elements have been inserted, adopt the name specified in the corresponding attribute of the
      // root element
      if (const char * name = other_root->Attribute(ROOT_TREE_ATTRIBUTE_NAME)) setRootTreeName(name);
    }
  } else if (strcmp(other_root->Name(), TREE_ELEMENT_NAME) == 0) {
    // Allow a single behavior tree without <root> element
    RootElement()->InsertEndChild(other_root->DeepClone(this));
  } else {
    throw exceptions::TreeDocumentError(
      "Cannot merge trees: Root element of other document must either be <" + std::string(ROOT_ELEMENT_NAME) +
      "> or <" + TREE_ELEMENT_NAME + ">.");
  }

  if (adopt_root_tree && other_tree_names.size() == 1) {
    // If there is only one tree element, we change the root tree to the respective tree after (!) all tree elements
    // have been inserted
    setRootTreeName(other_tree_names[0]);
  }

  return *this;
}

TreeDocument & TreeDocument::mergeString(const std::string & tree_str, bool adopt_root_tree)
{
  XMLDocument other_doc;
  if (other_doc.Parse(tree_str.c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
    throw exceptions::TreeDocumentError("Cannot merge tree document from string: " + std::string(other_doc.ErrorStr()));
  }
  return merge(other_doc, adopt_root_tree);
}

TreeDocument & TreeDocument::mergeFile(const std::string & path, bool adopt_root_tree)
{
  XMLDocument other_doc;
  if (other_doc.LoadFile(path.c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
    throw exceptions::TreeDocumentError("Cannot create tree document from file " + path + ": " + other_doc.ErrorStr());
  }
  return merge(other_doc, adopt_root_tree);
}

TreeDocument & TreeDocument::mergeResource(const TreeResource & resource, bool adopt_root_tree)
{
  return mergeFile(resource.tree_file_path_, adopt_root_tree);
}

std::vector<std::string> TreeDocument::getAllTreeNames() const { return getAllTreeNamesImpl(*this); }

bool TreeDocument::isExistingTreeName(const std::string & tree_name) const
{
  if (auto_apms_util::contains(getAllTreeNames(), tree_name)) return true;
  return false;
}

TreeDocument & TreeDocument::setRootTreeName(const std::string & tree_name)
{
  if (tree_name.empty()) {
    throw exceptions::TreeDocumentError("Cannot set root tree name with empty string.");
  }
  if (!isExistingTreeName(tree_name)) {
    throw exceptions::TreeDocumentError(
      "Cannot make tree with name '" + tree_name + "' the root tree because it doesn't exist.");
  }
  RootElement()->SetAttribute(ROOT_TREE_ATTRIBUTE_NAME, tree_name.c_str());
  return *this;
}

bool TreeDocument::hasRootTreeName() const
{
  if (RootElement()->Attribute(ROOT_TREE_ATTRIBUTE_NAME)) return true;
  return false;
}

std::string TreeDocument::getRootTreeName() const
{
  if (const auto tree_name = RootElement()->Attribute(ROOT_TREE_ATTRIBUTE_NAME)) return tree_name;
  throw exceptions::TreeDocumentError(
    "Cannot get root tree name because the document's root element has no attribute '" +
    std::string(ROOT_TREE_ATTRIBUTE_NAME) + "'.");
}

std::string TreeDocument::str() const
{
  tinyxml2::XMLPrinter printer;
  Print(&printer);
  return printer.CStr();
}

TreeDocument & TreeDocument::removeTreeWithName(const std::string & tree_name)
{
  RootElement()->DeleteChild(getXMLElementForTreeWithName(tree_name));
  return *this;
}

TreeDocument & TreeDocument::reset()
{
  Clear();
  setUpTreeDocument(*this);
  return *this;
}

TreeDocument & TreeDocument::reset(const TreeDocument & new_doc)
{
  reset();
  return merge(new_doc);
}

TreeDocument::XMLElement * TreeDocument::getXMLElementForTreeWithName(const std::string & tree_name)
{
  if (!isExistingTreeName(tree_name)) {
    throw exceptions::TreeDocumentError("Cannot get tree with name '" + tree_name + "' because it doesn't exist.");
  }
  XMLElement * ele = RootElement()->FirstChildElement(TREE_ELEMENT_NAME);
  while (ele && !ele->Attribute(TREE_NAME_ATTRIBUTE_NAME, tree_name.c_str())) {
    ele = ele->NextSiblingElement();
  }
  if (!ele) {
    throw std::logic_error(
      "Unexpected error trying to get tree element with name '" + tree_name +
      "'. Since isExistingTreeName() returned true, there MUST be a corresponding element.");
  }
  return ele;
}

}  // namespace auto_apms_behavior_tree::core