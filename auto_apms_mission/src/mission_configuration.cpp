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

#include "auto_apms_mission/mission_configuration.hpp"

#include <filesystem>

#include "ament_index_cpp/get_resource.hpp"
#include "auto_apms_util/exceptions.hpp"
#include "auto_apms_util/resource.hpp"
#include "auto_apms_util/string.hpp"

namespace auto_apms_mission
{
const std::string MissionConfiguration::YAML_KEY_BRINGUP = "BRINGUP";
const std::string MissionConfiguration::YAML_KEY_MISSION = "MISSION";
const std::string MissionConfiguration::YAML_KEY_CONTINGENCY = "CONTINGENCY";
const std::string MissionConfiguration::YAML_KEY_SHUTDOWN = "SHUTDOWN";

MissionConfiguration MissionConfiguration::fromResourceIdentity(const std::string identity)
{
  const auto tokens = auto_apms_util::splitString(identity, "::", false);
  std::string package_name = "";
  std::string file_name;
  switch (tokens.size()) {
    case 1:
      file_name = tokens[0];
      break;
    case 2:
      package_name = tokens[0];
      file_name = tokens[1];
      break;
    default:
      throw auto_apms_util::exceptions::ResourceIdentityFormatError(
        "Mission configuration resource identity string '" + identity +
        "' has wrong format. Must be '<package_name>::<file_name>'.");
  }

  std::set<std::string> search_packages;
  if (!package_name.empty()) {
    search_packages.insert(package_name);
  } else {
    search_packages =
      auto_apms_util::getPackagesWithResourceType(_AUTO_APMS_MISSION__RESOURCE_TYPE_NAME__MISSION_CONFIG);
  }

  std::vector<std::string> matching_file_paths;
  for (const auto & p : search_packages) {
    std::string content;
    std::string base_path;
    if (ament_index_cpp::get_resource(_AUTO_APMS_MISSION__RESOURCE_TYPE_NAME__MISSION_CONFIG, p, content, &base_path)) {
      std::vector<std::string> lines = auto_apms_util::splitString(content, "\n");
      for (const std::string & line : lines) {
        if (std::filesystem::path(line).stem() == std::filesystem::path(file_name).stem()) {
          matching_file_paths.push_back(base_path + "/" + line);
        }
      }
    }
  }

  if (matching_file_paths.empty()) {
    throw auto_apms_util::exceptions::ResourceError(
      "No mission configuration resource was found using identity '" + identity + "'.");
  }
  if (matching_file_paths.size() > 1) {
    throw auto_apms_util::exceptions::ResourceError(
      "There are multiple mission configuration resources with file name '" + file_name + "'.");
  }
  return fromFile(matching_file_paths[0]);
}
}  // namespace auto_apms_mission