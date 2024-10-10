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

#include <tinyxml2.h>

#include "auto_apms/behavior_tree/node_plugin_manifest.hpp"
#include "auto_apms/behavior_tree/resources.hpp"
#include "behaviortree_cpp/bt_factory.h"

namespace auto_apms {

class BehaviorTree
{
    static constexpr char MAIN_TREE_ATTRIBUTE_NAME[] = "main_tree_to_execute";

   public:
    using Resource = BehaviorTreeResource;
    using NodePluginManifest = detail::BTNodePluginManifest;

    BehaviorTree(const std::string& file_path);
    BehaviorTree(const Resource& resource);

    static BT::Tree Create(const std::string& tree_str,
                           const std::string& main_id,
                           BT::BehaviorTreeFactory& factory,
                           BT::Blackboard::Ptr parent_blackboard_ptr = nullptr);

    static BT::Tree Create(rclcpp::Node::SharedPtr node_ptr,
                           const Resource& resource,
                           const std::string& main_id,
                           BT::BehaviorTreeFactory& factory,
                           BT::Blackboard::Ptr parent_blackboard_ptr = nullptr);

    static BT::Tree Create(rclcpp::Node::SharedPtr node_ptr,
                           const Resource& resource,
                           const std::string& main_id,
                           BT::Blackboard::Ptr parent_blackboard_ptr = nullptr);

    BT::Tree Create(rclcpp::Node::SharedPtr node_ptr,
                    BT::BehaviorTreeFactory& factory,
                    BT::Blackboard::Ptr parent_blackboard_ptr = nullptr) const;

    BT::Tree Create(rclcpp::Node::SharedPtr node_ptr, BT::Blackboard::Ptr parent_blackboard_ptr = nullptr) const;

    std::string GetMainID() const;

    BehaviorTree& SetMainID(const std::string& main_id);

    std::string WriteToString() const;

   private:
    NodePluginManifest node_plugin_manifest_;
    tinyxml2::XMLDocument doc_;
};

}  // namespace auto_apms