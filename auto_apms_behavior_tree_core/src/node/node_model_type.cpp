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

#include "auto_apms_behavior_tree_core/node/node_model_type.hpp"

#include "auto_apms_behavior_tree_core/builder.hpp"
#include "auto_apms_behavior_tree_core/exceptions.hpp"

namespace auto_apms_behavior_tree
{
namespace model
{

BT::NodeType SubTree::type() { return BT::NodeType::SUBTREE; }

std::string SubTree::name() { return core::TreeDocument::SUBTREE_ELEMENT_NAME; }

std::string SubTree::getRegistrationName() const { return name(); }

SubTree & SubTree::setPreCondition(BT::PreCond type, const core::Script & script)
{
  LeafNodeModelType::setPreCondition(type, script);
  return *this;
}

SubTree & SubTree::setPostCondition(BT::PostCond type, const core::Script & script)
{
  LeafNodeModelType::setPostCondition(type, script);
  return *this;
}

SubTree & SubTree::set_auto_remap(bool val)
{
  setPorts({{"_autoremap", BT::toStr(val)}});
  return *this;
}

bool SubTree::get_auto_remap() const { return BT::convertFromString<bool>(getPorts().at("_autoremap")); }

}  // namespace model
}  // namespace auto_apms_behavior_tree
