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

#include "auto_apms_behavior_tree/executor_base.hpp"

namespace auto_apms_behavior_tree {

BTExecutorBase::BTExecutorBase(const std::string& node_name, const rclcpp::NodeOptions& options)
    : node_ptr_{std::make_shared<rclcpp::Node>(node_name, options)}, param_lisenter_{node_ptr_}
{}

rclcpp::Node::SharedPtr BTExecutorBase::node() { return node_ptr_; }

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr BTExecutorBase::get_node_base_interface()
{
    return node_ptr_->get_node_base_interface();
}

BT::Blackboard::Ptr BTExecutorBase::global_blackboard() { return global_blackboard_ptr_; }

}  // namespace auto_apms_behavior_tree
