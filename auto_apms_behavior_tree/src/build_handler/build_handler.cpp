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

#include "auto_apms_behavior_tree/build_handler/build_handler.hpp"

#include "auto_apms_behavior_tree/exceptions.hpp"

namespace auto_apms_behavior_tree
{

TreeBuildHandler::TreeBuildHandler(rclcpp::Node::SharedPtr node_ptr)
: logger_(node_ptr->get_logger().get_child("tree_build_handler")), ros_node_wptr_(node_ptr)
{
}

rclcpp::Node::SharedPtr TreeBuildHandler::getNodePtr() const
{
  if (ros_node_wptr_.expired()) {
    throw std::runtime_error("TreeBuildHandler: Weak pointer to rclcpp::Node expired.");
  }
  return ros_node_wptr_.lock();
}

}  // namespace auto_apms_behavior_tree