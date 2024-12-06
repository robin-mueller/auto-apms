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

#include "auto_apms_behavior_tree/build_handler.hpp"
#include "auto_apms_behavior_tree/exceptions.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_resource.hpp"

namespace auto_apms_behavior_tree
{

class TreeFromResourceBuildHandler : public TreeBuildHandler
{
public:
  TreeFromResourceBuildHandler(rclcpp::Node::SharedPtr ros_node_ptr, NodeLoader::SharedPtr tree_node_loader_ptr)
  : TreeBuildHandler("tree_from_resource", ros_node_ptr, tree_node_loader_ptr),
    resource_doc_(core::TreeDocument::BTCPP_FORMAT_DEFAULT_VERSION, tree_node_loader_ptr)
  {
  }

  bool setBuildRequest(
    const std::string & build_request, const NodeManifest & /*node_manifest*/,
    const std::string & root_tree_name) override final
  {
    TreeResource::Identity resource_identity(build_request);
    TreeResource resource(resource_identity);

    // We don't want to set the root tree yet
    resource_doc_.reset().mergeResource(resource, false);

    if (const BT::Result res = resource_doc_.verify(); !res) {
      RCLCPP_WARN(logger_, "Tree verification failed: %s", res.error().c_str());
      return false;
    }

    // Try to determine root tree name
    std::string name = root_tree_name;
    if (root_tree_name.empty()) {
      try {
        name = resource.getRootTreeName();
      } catch (const auto_apms_util::exceptions::ResourceError & e) {
        RCLCPP_WARN(
          logger_,
          "Cannot determine root tree from tree resource identity '%s': You must either specify the root_tree_name "
          "argument with a non empty string or provide an identity that includes a tree name.",
          resource_identity.str().c_str());
        return false;
      }
    }
    try {
      resource_doc_.setRootTreeName(name);
    } catch (const exceptions::TreeDocumentError & e) {
      RCLCPP_WARN(logger_, "Cannot determine root tree: %s", e.what());
      return false;
    }
    return true;
  }

  TreeDocument::TreeElement buildTree(TreeBuilder & builder, TreeBlackboard & /*bb*/) override final
  {
    // Merge document and adopt root tree
    builder.mergeTreeDocument(resource_doc_, true);

    // Reset the local tree document, as the tree moved to the builder document
    resource_doc_.reset();

    // The document MUST have a root tree. We made sure of that during setBuildRequest
    return builder.getRootTree();
  }

private:
  core::TreeDocument resource_doc_;
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_DECLARE_BUILD_HANDLER(auto_apms_behavior_tree::TreeFromResourceBuildHandler)