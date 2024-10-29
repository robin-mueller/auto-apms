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
#include "auto_apms_behavior_tree/node/node_registration_interface.hpp"

namespace auto_apms_behavior_tree
{

using NodePluginClassLoader = pluginlib::ClassLoader<NodeRegistrationInterface>;

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Create an instance of pluginlib::ClassLoader specifically for loading installed behavior tree build director
 * plugins.
 * @param search_packages Packages to consider when searching for plugin resources. Leave empty to search in all
 * packages.
 * @return Initialized class loader.
 * @throws auto_apms_core::exceptions::ResourceNotFoundError if failed to find a pluginlib plugin
 * manifest file in a package specified in @p search_packages or if a `ament_index` resource marker file is invalid.
 */
NodePluginClassLoader makeNodeClassLoader(const std::set<std::string>& search_packages = {});

}  // namespace auto_apms_behavior_tree