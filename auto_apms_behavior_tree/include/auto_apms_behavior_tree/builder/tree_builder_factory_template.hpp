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

#include "auto_apms_behavior_tree/builder/tree_builder_factory_interface.hpp"

namespace auto_apms_behavior_tree
{

template <typename T>
class TreeBuilderFactoryTemplate : public TreeBuilderFactoryInterface
{
public:
  TreeBuilderFactoryTemplate() = default;
  virtual ~TreeBuilderFactoryTemplate() = default;

  std::shared_ptr<TreeBuilderBase> instantiateBuilder(const rclcpp::Node::SharedPtr node_ptr)
  {
    static_assert(
      std::is_convertible_v<T *, TreeBuilderBase *>,
      "Cannot convert T* to TreeBuilderBase*. Did you forget to specify the keyword 'public' when "
      "inheriting? --> class T : public TreeBuilderBase");
    return std::make_shared<T>(node_ptr);
  }
};

}  // namespace auto_apms_behavior_tree