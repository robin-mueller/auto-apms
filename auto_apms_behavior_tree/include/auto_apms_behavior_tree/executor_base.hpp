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

#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "executor_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace auto_apms_behavior_tree {

class BTExecutorBase
{
   public:
    BTExecutorBase(const std::string& node_name, const rclcpp::NodeOptions& options);

    /// Get the node's base interface.
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();

   protected:
    /// Get a pointer to the internal ROS2 node instance.
    rclcpp::Node::SharedPtr node();

    /**
     * @brief Get a pointer to the global blackboard instance.
     *
     * The global blackboard is used as a parent blackboard on every tree that is being created. This means, that all
     * entries of the global blackboard are publicly available to the trees at runtime.
     */
    BT::Blackboard::Ptr global_blackboard();

   private:
    rclcpp::Node::SharedPtr node_ptr_;
    executor_params::ParamListener param_lisenter_;
    BT::Blackboard::Ptr global_blackboard_ptr_;
};

}  // namespace auto_apms_behavior_tree
