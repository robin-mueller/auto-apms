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
#include "auto_apms_behavior_tree/node/plugin_base.hpp"

namespace auto_apms_behavior_tree
{

using BTNodePluginClassLoader = pluginlib::ClassLoader<BTNodePluginBase>;

/**
 * @brief Create an instance of pluginlib::ClassLoader specifically for loading installed behavior tree node plugin
 * resources.
 * @ingroup auto_apms_behavior_tree
 * @param package_names Packages to consider when searching for resources. Leave empty to search in all packages.
 * @return pluginlib::ClassLoader object.
 */
std::shared_ptr<BTNodePluginClassLoader> MakeBTNodePluginClassLoader(const std::set<std::string>& package_names = {});

}  // namespace auto_apms_behavior_tree