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

#include "auto_apms_behavior_tree/builder/tree_build_director_factory.hpp"

namespace auto_apms_behavior_tree
{

template <typename T>
class TreeBuildDirectorFactoryTemplate : public TreeBuildDirectorFactory
{
public:
  TreeBuildDirectorFactoryTemplate() = default;
  virtual ~TreeBuildDirectorFactoryTemplate() = default;

  std::shared_ptr<TreeBuildDirectorBase> createBuildDirector(const rclcpp::Node::SharedPtr node_ptr)
  {
    static_assert(std::is_convertible_v<T*, TreeBuildDirectorBase*>,
                  "Cannot convert T* to TreeBuildDirectorBase*. Did you forget to specify the public keyword when "
                  "inheriting? (class T : public TreeBuildDirectorBase)");
    return std::make_shared<T>(node_ptr);
  }
};

}  // namespace auto_apms_behavior_tree