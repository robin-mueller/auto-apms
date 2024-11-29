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

class TreeResourceBuildHandler : public TreeBuildHandler
{
public:
  using TreeBuildHandler::TreeBuildHandler;

  bool setBuildRequest(const std::string & build_request, const std::string & root_tree_name) override final
  {
    core::TreeResourceIdentity resource_identity(build_request);
    resource_ptr_ = std::make_unique<core::TreeResource>(resource_identity);

    // We don't want to set the root tree yet
    resource_doc_.reset().mergeResource(*resource_ptr_, false);

    // Try to set root tree name
    std::string name = root_tree_name;
    if (root_tree_name.empty()) {
      try {
        name = resource_ptr_->getRootTreeName();
      } catch (const auto_apms_util::exceptions::ResourceError & e) {
        // Cannot determine root tree
        RCLCPP_WARN(
          logger_,
          "Cannot determine root tree from tree resource identity '%s'. You must either specify the root_tree_name "
          "argument with a non empty string or provide an identity that includes a tree name.",
          resource_identity.str().c_str());
        return false;
      }
    }
    resource_doc_.setRootTreeName(name);
    return true;
  }

  TreeElement buildTree(TreeBuilder & builder, TreeBlackboard & /*bb*/) override final
  {
    if (!resource_ptr_) {
      throw exceptions::TreeBuildError("TreeResourceBuildHandler - resource_ptr_ is nullptr.");
    }

    // Merge document and adopt root tree
    builder.mergeTreeDocument(resource_doc_, true);

    // Reset the local tree document, as the tree moved to the builder document
    resource_doc_.reset();

    // Load all node plugins associated with the resource
    builder.makeNodesAvailable(resource_ptr_->getNodeManifest());
    return builder.getRootTree();
  }

private:
  core::TreeDocument resource_doc_;
  std::unique_ptr<core::TreeResource> resource_ptr_;
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_BUILD_HANDLER(auto_apms_behavior_tree::TreeResourceBuildHandler)