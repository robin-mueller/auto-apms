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

#include "auto_apms_interfaces/action/land.hpp"

#include "auto_apms_behavior_tree_core/node.hpp"

namespace auto_apms_px4
{

class LandAction : public auto_apms_behavior_tree::core::RosActionNode<auto_apms_interfaces::action::Land>
{
public:
  using RosActionNode::RosActionNode;
};

}  // namespace auto_apms_px4

AUTO_APMS_BEHAVIOR_TREE_DECLARE_NODE(auto_apms_px4::LandAction)
