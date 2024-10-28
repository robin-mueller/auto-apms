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

#include "pluginlib/class_loader.hpp"
#include "auto_apms_behavior_tree/builder/tree_build_director_factory.hpp"

namespace auto_apms_behavior_tree
{

using TreeBuildDirectorClassLoader = pluginlib::ClassLoader<TreeBuildDirectorFactory>;

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Create an instance of pluginlib::ClassLoader specifically for loading installed behavior tree build directors.
 * @param package_names Packages to consider when searching for resources. Leave empty to search in all packages.
 * @return pluginlib::ClassLoader object.
 */
std::shared_ptr<TreeBuildDirectorClassLoader>
MakeTreeBuildDirectorClassLoader(const std::set<std::string>& package_names = {});

}  // namespace auto_apms_behavior_tree