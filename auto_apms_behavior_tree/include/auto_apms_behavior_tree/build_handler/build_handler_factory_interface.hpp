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

#include "auto_apms_behavior_tree/build_handler/build_handler.hpp"

namespace auto_apms_behavior_tree
{

class TreeBuildHandlerFactoryInterface
{
public:
  TreeBuildHandlerFactoryInterface() = default;
  virtual ~TreeBuildHandlerFactoryInterface() = default;

  virtual TreeBuildHandler::SharedPtr makeShared(
    rclcpp::Node::SharedPtr ros_node_ptr, core::NodeRegistrationLoader::SharedPtr tree_node_loader_ptr) = 0;

  virtual TreeBuildHandler::UniquePtr makeUnique(
    rclcpp::Node::SharedPtr ros_node_ptr, core::NodeRegistrationLoader::SharedPtr tree_node_loader_ptr) = 0;
};

}  // namespace auto_apms_behavior_tree