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

#include <set>
#include <string>

namespace auto_apms_core
{

/**
 * @brief Collect all package names that register a certain type of `ament_index` resources.
 *
 * \note Resources are not available until the respective ROS2 package is installed.
 *
 * @ingroup auto_apms_core
 * @param resource_type Name of the resource type.
 * @return Package names.
 * @throws exceptions::ResourceNotFoundError if no resources of type @p resource_type were found.
 */
std::set<std::string> GetAllPackagesWithResource(const std::string& resource_type);

}  // namespace auto_apms_core
