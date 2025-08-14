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
#include "auto_apms_behavior_tree_core/exceptions.hpp"

namespace auto_apms_behavior_tree
{

/**
 * @brief Standard build handler for building a behavior tree directly from an XML string.
 *
 * This build handler requires the user to encode the XML of a behavior tree and pass it as the build request.
 * This may be done for example using TreeDocument::writeToString.
 */
class TreeFromStringBuildHandler : public TreeBuildHandler
{
public:
  TreeFromStringBuildHandler(rclcpp::Node::SharedPtr ros_node_ptr, NodeLoader::SharedPtr tree_node_loader_ptr)
  : TreeBuildHandler("tree_from_string", ros_node_ptr, tree_node_loader_ptr),
    working_doc_(TreeDocument::BTCPP_FORMAT_DEFAULT_VERSION, tree_node_loader_ptr)
  {
  }

  bool setBuildRequest(
    const std::string & build_request, const std::string & entrypoint,
    const NodeManifest & node_manifest) override final
  {
    // Adopt the root tree if specified
    working_doc_.reset().mergeString(build_request, true);
    working_doc_.registerNodes(node_manifest, false);

    if (const BT::Result res = working_doc_.verify(); !res) {
      RCLCPP_WARN(logger_, "Tree verification failed: %s", res.error().c_str());
      return false;
    }

    // Try to determine root tree name
    if (entrypoint.empty()) {
      if (!working_doc_.hasRootTreeName()) {
        RCLCPP_WARN(
          logger_,
          "Cannot determine root tree: You must either encode the root tree within the tree XML or provide a non-empty "
          "name using the entrypoint argument.");
        return false;
      }
    } else {
      try {
        working_doc_.setRootTreeName(entrypoint);
      } catch (const exceptions::TreeDocumentError & e) {
        RCLCPP_WARN(logger_, "Cannot determine root tree: %s", e.what());
        return false;
      }
    }
    return true;
  }

  TreeDocument::TreeElement buildTree(TreeDocument & doc, TreeBlackboard & /*bb*/) override final
  {
    // Merge document and adopt root tree
    doc.mergeTreeDocument(working_doc_, true);

    // Reset the local tree document, as the tree moved to doc
    working_doc_.reset();

    // The document MUST have a root tree. We made sure of that during setBuildRequest
    return doc.getRootTree();
  }

private:
  core::TreeDocument working_doc_;
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_BUILD_HANDLER(auto_apms_behavior_tree::TreeFromStringBuildHandler)