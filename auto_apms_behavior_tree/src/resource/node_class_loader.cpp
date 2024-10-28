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

#include "auto_apms_behavior_tree/resource/node_class_loader.hpp"
#include "auto_apms_core/util/split.hpp"
#include "auto_apms_core/resources.hpp"

namespace auto_apms_behavior_tree
{

std::shared_ptr<BTNodePluginClassLoader> MakeBTNodePluginClassLoader(const std::set<std::string>& package_names)
{
  std::vector<std::string> xml_paths;
  for (const auto& package_name : package_names.empty() ? auto_apms_core::GetAllPackagesWithResource(
                                                              _AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__NODE) :
                                                          package_names)
  {
    std::string content;
    std::string base_path;
    if (ament_index_cpp::get_resource(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__NODE, package_name, content,
                                      &base_path))
    {
      std::vector<std::string> paths = auto_apms_core::util::SplitString(content, "\n", false);
      if (paths.size() != 1)
      {
        throw std::runtime_error("Invalid behavior tree node plugin resource file (Package: '" + package_name + "').");
      }
      xml_paths.push_back(base_path + '/' + paths[0]);
    }
  }
  return std::make_shared<BTNodePluginClassLoader>("auto_apms_behavior_tree",
                                                   "auto_apms_behavior_tree::BTNodePluginBase", "", xml_paths);
}

}  // namespace auto_apms_behavior_tree