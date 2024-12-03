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
  if (!ele_ptr) {
    throw exceptions::TreeDocumentError("Cannot create an instance of NodeElement with ele_ptr=nullptr.");
  }

  // We tolerate builder_ptr being nullptr here but keep in mind that the program will segfault if any methods relying
  // on the builder are called later
  std::vector<TreeBuilder::NodePortInfo> port_infos;
  if (builder_ptr_) {
    const TreeBuilder::NodeModelMap model = builder_ptr_->getNodeModel(true);
    TreeBuilder::NodeModelMap::const_iterator it = model.find(ele_ptr_->Name());
    if (it != model.end()) port_infos = it->second.port_infos;
  }

  // There should always be a corresponding element in the nodes model. However, if the constructor is called e.g.
  // before the builder loaded the respective plugin, that's not the case and setPorts() won't work as expected.
  // Therefore, we must ensure that all factory methods for a NodeElement verify that the node is known to the builder
  // before creating an instance. NOTE: We must not throw, since the constructor must also succeed for the derived
  // TreeElement class for which there is no node model and setPorts() is deleted anyways.
  if (!port_infos.empty()) {
    for (const TreeBuilder::NodePortInfo & port_info : port_infos) {
      port_keys_.push_back(port_info.port_name);
      if (!port_info.port_default.empty()) port_default_values_[port_info.port_name] = port_info.port_default;
    }
  }
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertNode(
  const std::string & node_name, const NodeElement * before_this)
{
  if (const std::set<std::string> names = builder_ptr_->getAvailableNodeNames(true);
      names.find(node_name) == names.end()) {
    throw exceptions::TreeDocumentError(
      "Cannot insert unkown node <" + node_name +
      ">. Before inserting a new node, the associated builder must load and register the corresponding behavior tree "
      "node plugin. Consider using a signature of insertNode() that does this automatically.");
  }
  XMLElement * ele = builder_ptr_->doc_.NewElement(node_name.c_str());
  return insertBeforeImpl(before_this, ele);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertNode(
  const std::string & node_name, const NodeRegistrationOptions & registration_options, const NodeElement * before_this)
{
  // Check if the node name is known. If the developer provided registration options for a node that has the same name
  // as one of the native ones, we enter loadNodes() and throw these names are reserved (Therefore we set include_native
  // to false).
  if (const std::set<std::string> names = builder_ptr_->getAvailableNodeNames(false);
      names.find(node_name) == names.end()) {
    builder_ptr_->loadNodes(NodeManifest({{node_name, registration_options}}));
  }
  return insertNode(node_name, before_this);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertNode(
  const NodeModelType & model, const NodeElement * before_this)
{
  // Check if the node model is known. Since we also provide node models for the native nodes, we may include these in
  // the list of names to search (Therefore we set include_native
  // to true).
  const std::string node_name = model.getRegistrationName();
  if (const std::set<std::string> names = builder_ptr_->getAvailableNodeNames(true);
      names.find(node_name) == names.end()) {
    builder_ptr_->loadNodes(NodeManifest({{node_name, model.getRegistrationOptions()}}));
  }
  // We insert and additionally add the ports using the model's internal data.
  return insertNode(node_name, before_this).setPorts(model.getPortValues(), true);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertSubTreeNode(
  const std::string & tree_name, const NodeElement * before_this)
{
  return insertNode(SUBTREE_ELEMENT_NAME, before_this).setPorts({{TREE_NAME_ATTRIBUTE_NAME, tree_name}}, false);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTree(
  const TreeElement & tree, const NodeElement * before_this)
{
  // When inserting an instance of TreeElement directly, we can assume that the availability of the child nodes has
  // already been verified.
  const XMLElement * root_child = tree.ele_ptr_->FirstChildElement();
  if (!root_child) {
    throw exceptions::TreeDocumentError(
      "Cannot insert tree element '" + tree.getFullyQualifiedName() + "' because it has no child nodes.");
  }
  if (root_child->NextSibling()) {
    throw exceptions::TreeDocumentError(
      "Cannot insert tree element '" + tree.getFullyQualifiedName() + "' because it has more than one child node.");
  }
  return insertBeforeImpl(before_this, root_child->DeepClone(&builder_ptr_->doc_)->ToElement());
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromDocument(
  const TreeDocument & other, const std::string & tree_name, const NodeElement * before_this)
{
  const std::vector<std::string> other_tree_names = other.getAllTreeNames();
  if (other_tree_names.empty()) {
    throw exceptions::TreeDocumentError(
      "Cannot insert tree '" + tree_name + "' because other document has no <" + TREE_ELEMENT_NAME + "> elements.");
  }
  if (!auto_apms_util::contains(other_tree_names, tree_name)) {
    throw exceptions::TreeDocumentError(
      "Cannot insert tree '" + tree_name + "' because other document doesn't specify a tree with that name.");
  }

  // List including the target tree name and the names of its dependencies (Trees required by SubTree node)
  std::set<std::string> required_tree_names = {tree_name};

  // Find all subtree nodes and push back the associated tree to required_tree_names.
  // NOTE: All tree elements within a TreeDocument instance always have exactly one child, so we must only search the
  // children of the root child.
  TreeDocument temp_doc;  // Temporary non const working document
  other.DeepCopy(&temp_doc);
  const NodeElement temp(nullptr, temp_doc.getXMLElementForTreeWithName(tree_name)->FirstChildElement());
  temp.deepApply(ConstDeepApplyCallback([&required_tree_names](const NodeElement & ele) {
    if (ele.getRegistrationName() == SUBTREE_ELEMENT_NAME) {
      if (const char * name = ele.ele_ptr_->Attribute(TREE_NAME_ATTRIBUTE_NAME)) required_tree_names.insert(name);
    }
    // The return value doesn't matter here
    return false;
  }));

  // Verify that all child nodes of the trees listed in required_tree_names are known to the builder
  ConstDeepApplyCallback apply = [available_names =
                                    builder_ptr_->getAvailableNodeNames(true)](const NodeElement & ele) {
    return available_names.find(ele.getRegistrationName()) == available_names.end();
  };
  for (const std::string & name : required_tree_names) {
    const NodeElement ele(nullptr, temp_doc.getXMLElementForTreeWithName(name)->FirstChildElement());
    if (const std::vector<NodeElement> found = ele.deepApply(apply); !found.empty()) {
      std::vector<std::string> names;
      for (const NodeElement & ele : found) names.push_back(ele.getFullyQualifiedName());
      throw exceptions::TreeDocumentError(
        "Cannot insert tree '" + tree_name + "' because the following nodes found in tree '" + name +
        "' are unkown to the builder:\n\t- " + auto_apms_util::join(names, "\n\t- "));
    }
  }

  // Remove the data from each tree that the target tree doesn't directly depend on and merge the working document. This
  // approach ensures, that the verification logic inside merge() is applied to the working document.
  required_tree_names.erase(tree_name);  // Don't add the target tree twice
  for (const std::string & name : other_tree_names) {
    if (required_tree_names.find(name) == required_tree_names.end()) {
      // Remove if not required
      temp_doc.removeTreeWithName(name);
    }
  }
  builder_ptr_->doc_.merge(temp_doc, false);

  // Clone and insert the target tree if verification of dependency tree document succeeded
  return insertBeforeImpl(
    before_this,
    other.getXMLElementForTreeWithName(tree_name)->FirstChildElement()->DeepClone(&builder_ptr_->doc_)->ToElement());
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromDocument(
  const TreeDocument & other, const NodeElement * before_this)
{
  return insertTreeFromDocument(other, other.getRootTreeName(), before_this);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromString(
  const std::string & tree_str, const std::string & tree_name, const NodeElement * before_this)
{
  TreeDocument insert_doc;
  insert_doc.mergeString(tree_str);
  return insertTreeFromDocument(insert_doc, tree_name, before_this);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromString(
  const std::string & tree_str, const NodeElement * before_this)
{
  TreeDocument insert_doc;
  insert_doc.mergeString(tree_str);
  return insertTreeFromDocument(insert_doc, before_this);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromFile(
  const std::string & path, const std::string & tree_name, const NodeElement * before_this)
{
  TreeDocument insert_doc;
  insert_doc.mergeFile(path);
  return insertTreeFromDocument(insert_doc, tree_name, before_this);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromFile(
  const std::string & path, const NodeElement * before_this)
{
  TreeDocument insert_doc;
  insert_doc.mergeFile(path);
  return insertTreeFromDocument(insert_doc, before_this);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromResource(
  const TreeResource & resource, const std::string & tree_name, const NodeElement * before_this)
{
  TreeDocument insert_doc;
  insert_doc.mergeFile(resource.tree_file_path_);

  // We load all associated node plugins beforehand, so that the user doesn't have to do that manually. This means, that
  // also potentially unused nodes are available and registered with the factory. This seems unnecessary, but it's very
  // convenient and the performance probably doesn't suffer too much.
  builder_ptr_->loadNodes(resource.getNodeManifest(), false);
  return insertTreeFromDocument(insert_doc, tree_name, before_this);
}

TreeDocument::NodeElement TreeDocument::NodeElement::insertTreeFromResource(
  const TreeResource & resource, const NodeElement * before_this)
{
  return insertTreeFromResource(resource, resource.getRootTreeName(), before_this);
}

bool TreeDocument::NodeElement::hasChildren() const { return ele_ptr_->FirstChild() == nullptr ? false : true; }

TreeDocument::NodeElement TreeDocument::NodeElement::getFirstNode(const std::string & name) const
{
  if (name.empty()) return NodeElement(builder_ptr_, ele_ptr_->FirstChildElement());

  // If name is given, recursively search for the first node with this name
  ConstDeepApplyCallback apply = [name](const NodeElement & ele) {
    return ele.getRegistrationName() == name || ele.getName() == name;
  };
  if (const std::vector<NodeElement> found = deepApply(apply); !found.empty()) return found[0];

  // Cannot find node in children of this
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

TreeDocument::NodeElement & TreeDocument::NodeElement::setPorts(const PortValues & port_values, bool verify)
{
  const char * this_node_name = ele_ptr_->Name();
  std::vector<std::string> implemented_ports_keys = port_keys_;
  if (strcmp(this_node_name, SUBTREE_ELEMENT_NAME) == 0) {
    if (port_values.find(TREE_NAME_ATTRIBUTE_NAME) == port_values.end()) {
      throw exceptions::TreeDocumentError(
        "Cannot set ports for <" + std::string(SUBTREE_ELEMENT_NAME) + "> node, because required key '" +
        TREE_NAME_ATTRIBUTE_NAME + "' for identifying the associated tree is missing in port_values.");
    }
    if (const std::string & tree_name = port_values.at(TREE_NAME_ATTRIBUTE_NAME);
        !builder_ptr_->doc_.isExistingTreeName(tree_name)) {
      throw exceptions::TreeDocumentError(
        "Cannot set ports for <" + std::string(SUBTREE_ELEMENT_NAME) + "> node, because associated tree '" + tree_name +
        "' doesn't exist.");
    }

    // Tree name attribute is not implemented as a port, so we must add it to the vector manually.
    implemented_ports_keys.push_back(TREE_NAME_ATTRIBUTE_NAME);
  }

  // Verify port_values
  if (verify) {
    std::vector<std::string> unkown_keys;
    for (const auto & [key, _] : port_values) {
      if (!auto_apms_util::contains(implemented_ports_keys, key)) unkown_keys.push_back(key);
    }
    if (!unkown_keys.empty()) {
      throw exceptions::TreeDocumentError(
        "Cannot set ports. The following port keys are not implemented by node '" + std::string(this_node_name) +
        "' according to the internal model: [ " + auto_apms_util::join(unkown_keys, ", ") + " ].");
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
    } else if (port_default_values_.find(key) != port_default_values_.end()) {
      ele_ptr_->SetAttribute(key.c_str(), port_default_values_.at(key).c_str());
    }
    // If implemented port is neither specified in port_values nor has it a default value, ignore.
  }
  return *this;
}

TreeDocument::NodeElement & TreeDocument::NodeElement::setPreCondition(NodePreCondition type, const Script & script)
{
  ele_ptr_->SetAttribute(BT::toStr(type).c_str(), script.str().c_str());
  return *this;
}

TreeDocument::NodeElement & TreeDocument::NodeElement::setPostCondition(NodePostCondition type, const Script & script)
{
  ele_ptr_->SetAttribute(BT::toStr(type).c_str(), script.str().c_str());
  return *this;
}

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

const std::vector<TreeDocument::NodeElement> TreeDocument::NodeElement::deepApply(
  ConstDeepApplyCallback apply_callback) const
{
  std::vector<NodeElement> found;
  deepApplyImpl(*this, apply_callback, found);
  return found;
}

std::vector<TreeDocument::NodeElement> TreeDocument::NodeElement::deepApply(DeepApplyCallback apply_callback)
{
  std::vector<NodeElement> found;
  deepApplyImpl(*this, apply_callback, found);
  return found;
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

void TreeDocument::NodeElement::deepApplyImpl(
  const NodeElement & parent, ConstDeepApplyCallback apply_callback, std::vector<NodeElement> & vec)
{
  for (XMLElement * child = parent.ele_ptr_->FirstChildElement(); child != nullptr;
       child = child->NextSiblingElement()) {
    const NodeElement child_ele(parent.builder_ptr_, child);

    // Apply on current child element
    if (apply_callback(child_ele)) vec.push_back(child_ele);

    // Search children of child before evaluating the siblings
    deepApplyImpl(child_ele, apply_callback, vec);
  }
}

void TreeDocument::NodeElement::deepApplyImpl(
  NodeElement & parent, DeepApplyCallback apply_callback, std::vector<NodeElement> & vec)
{
  for (XMLElement * child = parent.ele_ptr_->FirstChildElement(); child != nullptr;
       child = child->NextSiblingElement()) {
    NodeElement child_ele(parent.builder_ptr_, child);

    // Apply on current child element
    if (apply_callback(child_ele)) vec.push_back(child_ele);

    // Search children of child before evaluating the siblings
    deepApplyImpl(child_ele, apply_callback, vec);
  }
}

TreeDocument::TreeElement::TreeElement(TreeBuilder * builder_ptr, XMLElement * ele_ptr)
: NodeElement(builder_ptr, ele_ptr)
{
  if (!ele_ptr->Attribute(TREE_NAME_ATTRIBUTE_NAME)) {
    throw exceptions::TreeDocumentError(
      "Cannot create tree element without a '" + std::string(TREE_NAME_ATTRIBUTE_NAME) + "' attribute.");
  }
}

std::string TreeDocument::TreeElement::getName() const
{
  if (const char * name = ele_ptr_->Attribute(TREE_NAME_ATTRIBUTE_NAME)) return name;
  return "unkown";
}

TreeDocument::TreeElement & TreeDocument::TreeElement::makeRoot()
{
  builder_ptr_->setRootTreeName(*this);
  return *this;
}

void setUpTreeDocument(tinyxml2::XMLDocument & doc, const std::string & format_version)
{
  tinyxml2::XMLElement * root_ele = doc.NewElement(TreeDocument::ROOT_ELEMENT_NAME);
  root_ele->SetAttribute(TreeDocument::BTCPP_FORMAT_ATTRIBUTE_NAME, format_version.c_str());
  doc.InsertFirstChild(root_ele);
}

TreeDocument::TreeDocument(const std::string & format_version) : XMLDocument(), format_version_(format_version)
{
  setUpTreeDocument(*this, format_version_);
}

TreeDocument::TreeDocument(const TreeDocument & other) : TreeDocument() { merge(other, true); }

TreeDocument & TreeDocument::operator=(const TreeDocument & other) { return merge(other, true); }

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
      "Cannot merge tree document: The following trees are already defined: [ " +
      auto_apms_util::join(common_tree_names, ", ") + " ].");
  }

  if (strcmp(other_root->Name(), ROOT_ELEMENT_NAME) == 0) {
    // Verify format
    if (const char * ver = other_root->Attribute(BTCPP_FORMAT_ATTRIBUTE_NAME)) {
      if (std::string(ver) != format_version_) {
        throw exceptions::TreeDocumentError(
          "Cannot merge tree document: Format of other document (" + std::string(BTCPP_FORMAT_ATTRIBUTE_NAME) + ": " +
          ver + ") is not compatible with this document (" + std::string(BTCPP_FORMAT_ATTRIBUTE_NAME) + ": " +
          format_version_ + ").");
      }
    } else {
      throw exceptions::TreeDocumentError(
        "Cannot merge tree document: Root element of other document doesn't have required attribute '" +
        std::string(BTCPP_FORMAT_ATTRIBUTE_NAME) + "'.");
    }

    // Iterate over all the children of other root
    for (const XMLElement * child = other_root->FirstChildElement(); child != nullptr;
         child = child->NextSiblingElement()) {
      // Disregard tree node model elements
      if (strcmp(child->Name(), TREE_NODE_MODEL_ELEMENT_NAME) == 0) continue;

      // Verify the structure of the behavior tree element
      if (strcmp(child->Name(), TREE_ELEMENT_NAME) == 0) {
        const char * tree_id = child->Attribute(TREE_NAME_ATTRIBUTE_NAME);
        if (!tree_id) {
          throw exceptions::TreeDocumentError(
            "Cannot merge tree document: Found a <" + std::string(TREE_ELEMENT_NAME) +
            "> element that doesn't specify the required attribute '" + TREE_NAME_ATTRIBUTE_NAME + "'.");
        }
        const XMLElement * tree_root_child = child->FirstChildElement();
        if (!tree_root_child) {
          throw exceptions::TreeDocumentError(
            "Cannot merge tree document: Tree '" + std::string(tree_id) + "' has no child nodes.");
        }
        if (tree_root_child->NextSibling()) {
          throw exceptions::TreeDocumentError(
            "Cannot merge tree document: Tree '" + std::string(tree_id) + "' has more than one child node.");
        }
      }

      // If tree element is valid, append to this document
      RootElement()->InsertEndChild(child->DeepClone(this));
    }

    if (adopt_root_tree) {
      // After (!) all tree elements have been inserted, adopt the name specified in the corresponding attribute of the
      // root element
      if (const char * name = other_root->Attribute(ROOT_TREE_ATTRIBUTE_NAME)) setRootTreeName(name);
    }
  } else if (strcmp(other_root->Name(), TREE_ELEMENT_NAME) == 0) {
    // Allow a single behavior tree without <root> element.
    // We assume that the the format complies with the version configured at construction time
    RootElement()->InsertEndChild(other_root->DeepClone(this));
  } else {
    throw exceptions::TreeDocumentError(
      "Cannot merge tree document: Root element of other document must either be <" + std::string(ROOT_ELEMENT_NAME) +
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
  setUpTreeDocument(*this, format_version_);
  return *this;
}

TreeDocument & TreeDocument::reset(const TreeDocument & new_doc)
{
  reset();
  return merge(new_doc);
}

const TreeDocument::XMLElement * TreeDocument::getXMLElementForTreeWithName(const std::string & tree_name) const
{
  return getXMLElementForTreeWithNameImpl<const XMLElement *>(*this, tree_name);
}

TreeDocument::XMLElement * TreeDocument::getXMLElementForTreeWithName(const std::string & tree_name)
{
  return getXMLElementForTreeWithNameImpl<XMLElement *>(*this, tree_name);
}

template <typename ReturnT, typename DocumentT>
ReturnT TreeDocument::getXMLElementForTreeWithNameImpl(DocumentT & doc, const std::string & tree_name)
{
  if (tree_name.empty()) {
    throw exceptions::TreeDocumentError("Cannot get tree with an empty name.");
  }
  if (!doc.isExistingTreeName(tree_name)) {
    throw exceptions::TreeDocumentError("Cannot get tree with name '" + tree_name + "' because it doesn't exist.");
  }
  auto child = doc.RootElement()->FirstChildElement(TreeDocument::TREE_ELEMENT_NAME);
  while (child && !child->Attribute(TreeDocument::TREE_NAME_ATTRIBUTE_NAME, tree_name.c_str())) {
    child = child->NextSiblingElement();
  }
  if (!child) {
    throw std::logic_error(
      "Unexpected error trying to get tree element with name '" + tree_name +
      "'. Since isExistingTreeName() returned true, there MUST be a corresponding element.");
  }
  return child;
}

}  // namespace auto_apms_behavior_tree::core