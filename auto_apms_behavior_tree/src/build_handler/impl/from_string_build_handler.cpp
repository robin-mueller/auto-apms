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

class TreeFromResourceBuildHandler : public TreeBuildHandler
{
public:
  using TreeBuildHandler::TreeBuildHandler;

  bool setBuildRequest(const std::string & build_request, const std::string & root_tree_name) override final
  {
    // Adopt the root tree if specified
    resource_doc_.reset().mergeString(build_request, true);

    // Try to determine root tree name
    if (root_tree_name.empty()) {
      if (!resource_doc_.hasRootTreeName()) {
        RCLCPP_WARN(
          logger_,
          "Cannot determine root tree: You must either encode the root tree within the tree XML or provide a non-empty "
          "name using the root_tree_name argument.");
        return false;
      }
    } else {
      try {
        resource_doc_.setRootTreeName(root_tree_name);
      } catch (const exceptions::TreeDocumentError & e) {
        RCLCPP_WARN(logger_, "Cannot determine root tree: %s", e.what());
        return false;
      }
    }
    return true;
  }

  TreeElement buildTree(TreeBuilder & builder, TreeBlackboard & /*bb*/) override final
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

AUTO_APMS_BEHAVIOR_TREE_REGISTER_BUILD_HANDLER(auto_apms_behavior_tree::TreeFromResourceBuildHandler)