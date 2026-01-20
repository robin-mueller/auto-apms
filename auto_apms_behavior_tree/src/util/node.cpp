// Copyright 2024 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "auto_apms_behavior_tree/util/node.hpp"

namespace auto_apms_behavior_tree
{
model::StartExecutor insertStartExecutorFromString(
  core::TreeDocument::NodeElement & parent, const std::string & tree_str,
  const core::TreeDocument::NodeElement * before_this)
{
  model::StartExecutor ele = parent.insertNode<model::StartExecutor>(before_this);
  ele.set_build_request(tree_str);
  ele.set_build_handler("auto_apms_behavior_tree::TreeFromStringBuildHandler");
  return ele;
}

model::StartExecutor insertStartExecutorFromString(
  core::TreeDocument::NodeElement & parent, const core::TreeDocument::TreeElement & tree,
  const core::TreeDocument::NodeElement * before_this)
{
  return insertStartExecutorFromString(parent, tree.writeToString(), before_this)
    .set_node_manifest(tree.getRequiredNodeManifest().encode());
}

model::StartExecutor insertStartExecutorFromResource(
  core::TreeDocument::NodeElement & parent, const core::TreeResourceIdentity & identity,
  const core::TreeDocument::NodeElement * before_this)
{
  model::StartExecutor ele = parent.insertNode<model::StartExecutor>(before_this);
  ele.set_build_request(identity.str());
  ele.set_build_handler("auto_apms_behavior_tree::TreeFromResourceBuildHandler");
  return ele;
}

}  // namespace auto_apms_behavior_tree