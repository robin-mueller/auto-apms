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

#include "auto_apms_behavior_tree/builtin_nodes.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_resource.hpp"

namespace auto_apms_behavior_tree
{

model::StartExecutor insertStartExecutorFromString(
  core::TreeDocument::NodeElement & parent, const std::string & tree_str,
  const core::TreeDocument::NodeElement * before_this = nullptr);

model::StartExecutor insertStartExecutorFromString(
  core::TreeDocument::NodeElement & parent, const core::TreeDocument::TreeElement & tree,
  const core::TreeDocument::NodeElement * before_this = nullptr);

model::StartExecutor insertStartExecutorFromResource(
  core::TreeDocument::NodeElement & parent, const core::TreeResource & resource,
  const core::TreeDocument::NodeElement * before_this = nullptr);

}  // namespace auto_apms_behavior_tree