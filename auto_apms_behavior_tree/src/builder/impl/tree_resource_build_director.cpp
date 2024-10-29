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

#include "auto_apms_behavior_tree/tree_build_director.hpp"
#include "auto_apms_behavior_tree/resource/tree_resource.hpp"

namespace auto_apms_behavior_tree
{

class TreeResourceBuildDirector : public TreeBuildDirectorBase
{
public:
  using TreeBuildDirectorBase::TreeBuildDirectorBase;

  void setTreeIdentity(const std::string& identity) override final
  {
    resource_ptr_ = std::make_unique<TreeResource>(TreeResource::FromString(identity));
  }

  bool executeBuildSteps(TreeBuilder& builder) override final
  {
    if (!resource_ptr_)
      return false;
    builder.addTreeFromResource(*resource_ptr_, getNode());
    return true;
  }

private:
  std::unique_ptr<TreeResource> resource_ptr_;
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_BUILD_DIRECTOR(auto_apms_behavior_tree::TreeResourceBuildDirector)