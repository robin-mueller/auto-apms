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

#pragma once

#include "auto_apms/behavior_tree/node_loader.hpp"
#include "auto_apms/behavior_tree/resources.hpp"
#include "auto_apms/behavior_tree/tree_xml.hpp"
#include "behaviortree_cpp/bt_factory.h"

namespace auto_apms {

BT::Tree CreateBehaviorTree(BTNodePluginLoader& node_plugin_loader,
                            const std::string& tree_xml,
                            BT::BehaviorTreeFactory* const factory = nullptr,
                            BT::Blackboard::Ptr parent_blackboard_ptr = nullptr);

BT::Tree CreateBehaviorTree(rclcpp::Node::SharedPtr node_ptr,
                            const BehaviorTreeResource& resource,
                            BT::BehaviorTreeFactory* const factory = nullptr,
                            BT::Blackboard::Ptr parent_blackboard_ptr = nullptr);

}  // namespace auto_apms