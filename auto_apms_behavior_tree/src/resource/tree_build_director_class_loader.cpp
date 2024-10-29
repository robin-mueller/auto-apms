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

#include "auto_apms_behavior_tree/resource/tree_build_director_class_loader.hpp"
#include "auto_apms_core/resources.hpp"

namespace auto_apms_behavior_tree
{

TreeBuildDirectorClassLoader makeTreeBuildDirectorClassLoader(const std::set<std::string>& search_packages)
{
  return auto_apms_core::makePluginClassLoader<TreeBuildDirectorFactory>(
      "auto_apms_behavior_tree", "auto_apms_behavior_tree::TreBuildDirectorFactory",
      _AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__BUILD_DIRECTOR, search_packages);
}

}  // namespace auto_apms_behavior_tree
