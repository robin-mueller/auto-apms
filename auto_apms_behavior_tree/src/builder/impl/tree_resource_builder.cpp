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

#include "auto_apms_behavior_tree/builder.hpp"
#include "auto_apms_behavior_tree/resource/tree_resource.hpp"
#include "auto_apms_behavior_tree/exceptions.hpp"

namespace auto_apms_behavior_tree
{

class TreeResourceBuilder : public TreeBuilderBase
{
public:
  using TreeBuilderBase::TreeBuilderBase;

  bool setRequest(const std::string& request) override final
  {
    try
    {
      resource_ptr_ = std::make_unique<TreeResource>(TreeResource::fromString(request));
    }
    catch (const exceptions::ResourceIdentityFormatError& e)
    {
      RCLCPP_ERROR(getLogger(), "%s", e.what());
      return false;
    }
    catch (const auto_apms_util::exceptions::ResourceError& e)
    {
      RCLCPP_ERROR(getLogger(), "%s", e.what());
      return false;
    }
    return true;
  }

  void configureBuilder(TreeBuilder& builder) override final
  {
    if (!resource_ptr_)
      throw exceptions::TreeBuildError("TreeResourceBuilder - resource_ptr_ is nullptr.");
    builder.mergeTreesFromResource(*resource_ptr_, getNodePtr());
  }

private:
  std::unique_ptr<TreeResource> resource_ptr_;
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_BUILDER(auto_apms_behavior_tree::TreeResourceBuilder)