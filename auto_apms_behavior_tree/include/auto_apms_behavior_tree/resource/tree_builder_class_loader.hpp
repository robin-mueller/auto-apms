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

#include "auto_apms_util/resource.hpp"
#include "auto_apms_behavior_tree/builder/tree_builder_factory_interface.hpp"

namespace auto_apms_behavior_tree
{

/**
 * @ingroup auto_apms_behavior_tree
 * @brief A pluginlib::ClassLoader specifically for loading installed behavior tree builder plugins.
 */
class TreeBuilderClassLoader : public auto_apms_util::ResourceClassLoader<TreeBuilderFactoryInterface>
{
public:
  static const std::string BASE_PACKAGE_NAME;
  static const std::string BASE_CLASS_NAME;
  static const std::string RESOURCE_TYPE_NAME;

  /**
   * @brief TreeBuilderClassLoader constructor.
   * @param search_packages Packages to consider when searching for associated plugin resources. Leave empty to search
   * in all packages.
   * @throws auto_apms_util::exceptions::ResourceError if failed to find a pluginlib plugin
   * manifest file in a package specified in @p search_packages or if an `ament_index` resource marker file is invalid.
   */
  TreeBuilderClassLoader(const std::set<std::string>& search_packages = {});
};

}  // namespace auto_apms_behavior_tree