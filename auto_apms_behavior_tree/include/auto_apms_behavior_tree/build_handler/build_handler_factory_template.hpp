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

#include "auto_apms_behavior_tree/build_handler/build_handler_factory_interface.hpp"

namespace auto_apms_behavior_tree
{

template <typename T>
class TreeBuildHandlerFactoryTemplate : public TreeBuildHandlerFactoryInterface
{
  static_assert(
    std::is_convertible_v<T *, TreeBuildHandler *>,
    "Cannot convert T* to TreeBuildHandler*. Did you forget to specify the keyword 'public' when inheriting? --> class "
    "T : public TreeBuildHandler");

public:
  TreeBuildHandlerFactoryTemplate() = default;
  virtual ~TreeBuildHandlerFactoryTemplate() = default;

  TreeBuildHandler::SharedPtr makeShared(rclcpp::Node::SharedPtr node_ptr) override final
  {
    return std::make_shared<T>(node_ptr);
  }

  TreeBuildHandler::UniquePtr makeUnique(rclcpp::Node::SharedPtr node_ptr) override final
  {
    return std::make_unique<T>(node_ptr);
  }
};

}  // namespace auto_apms_behavior_tree