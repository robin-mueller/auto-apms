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

#include "auto_apms_core/resources.hpp"

#include "ament_index_cpp/get_resources.hpp"
#include "auto_apms_core/exceptions.hpp"

namespace auto_apms_core
{

std::set<std::string> GetAllPackagesWithResource(const std::string& resource_type)
{
  std::set<std::string> all_packages;
  for (const auto& [package_name, _] : ament_index_cpp::get_resources(resource_type))
  {
    all_packages.insert(package_name);
  }
  if (all_packages.empty())
  {
    throw exceptions::ResourceNotFoundError("No resources of type '" + resource_type +
                                            "' were found in the installed packages.");
  }
  return all_packages;
}

}  // namespace auto_apms_core
