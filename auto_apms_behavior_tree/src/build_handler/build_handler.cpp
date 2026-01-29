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

#include "auto_apms_behavior_tree/build_handler/build_handler.hpp"

#include "auto_apms_behavior_tree/exceptions.hpp"

namespace auto_apms_behavior_tree
{

TreeBuildHandler::TreeBuildHandler(
  const std::string & name, rclcpp::Node::SharedPtr ros_node_ptr, NodeLoader::SharedPtr tree_node_loader_ptr)
: logger_(ros_node_ptr->get_logger().get_child(name)),
  ros_node_wptr_(ros_node_ptr),
  tree_node_loader_ptr(tree_node_loader_ptr)
{
}

TreeBuildHandler::TreeBuildHandler(rclcpp::Node::SharedPtr ros_node_ptr, NodeLoader::SharedPtr tree_node_loader_ptr)
: TreeBuildHandler("tree_build_handler", ros_node_ptr, tree_node_loader_ptr)
{
}

bool TreeBuildHandler::setBuildRequest(
  const std::string & /*build_request*/, const std::string & /*entry_point*/, const NodeManifest & /*node_manifest*/)
{
  return true;
}

rclcpp::Node::SharedPtr TreeBuildHandler::getRosNodePtr() const
{
  if (ros_node_wptr_.expired()) {
    throw std::runtime_error("TreeBuildHandler: Weak pointer to rclcpp::Node expired.");
  }
  return ros_node_wptr_.lock();
}

TreeBuildHandler::NodeLoader::SharedPtr TreeBuildHandler::getNodeLoaderPtr() const { return tree_node_loader_ptr; }

}  // namespace auto_apms_behavior_tree