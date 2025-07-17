// Copyright 2025 Robin Müller
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

#include "auto_apms_behavior_tree_core/behavior.hpp"

namespace auto_apms_behavior_tree::core
{

BehaviorResourceIdentity::BehaviorResourceIdentity(const std::string & identity)
{
  std::string resource_part;
  if (std::size_t pos = identity.find(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP);
      pos == std::string::npos) {
    category_name = "";
    resource_part = identity;
  } else {
    category_name = identity.substr(0, pos);
    resource_part =
      identity.substr(pos + std::string(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP).size());
  }
  if (resource_part.empty()) {
    throw auto_apms_util::exceptions::ResourceIdentityFormatError(
      "Behavior resource identity string '" + identity + "' is invalid: Package and resource name must not be empty.");
  }
  if (std::size_t pos = resource_part.find(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP);
      pos == std::string::npos) {
    // If only a single token is given, assume it's behavior_alias
    package_name = "";
    behavior_alias = resource_part;
  } else {
    package_name = resource_part.substr(0, pos);
    behavior_alias =
      resource_part.substr(pos + std::string(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP).size());
  }
  if (package_name.empty() && behavior_alias.empty()) {
    throw auto_apms_util::exceptions::ResourceIdentityFormatError(
      "Behavior resource identity string '" + identity + "' is invalid. Package and resource name must not be empty.");
  }
}

BehaviorResourceIdentity::BehaviorResourceIdentity(const char * identity)
: BehaviorResourceIdentity(std::string(identity))
{
}

bool BehaviorResourceIdentity::operator==(const BehaviorResourceIdentity & other) const { return str() == other.str(); }

bool BehaviorResourceIdentity::operator<(const BehaviorResourceIdentity & other) const { return str() < other.str(); }

std::string BehaviorResourceIdentity::str() const
{
  std::string str;
  if (!category_name.empty()) {
    str += category_name + _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP;
  }
  str += package_name + _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP + behavior_alias;
  return str;
}

bool BehaviorResourceIdentity::empty() const { return package_name.empty() && behavior_alias.empty(); }

std::set<BehaviorResourceIdentity> getBehaviorResourceIdentities(
  const std::set<std::string> & include_categories, const std::set<std::string> & exclude_packages)
{
  std::set<BehaviorResourceIdentity> identities;
  for (const auto & p : auto_apms_util::getPackagesWithResourceType(
         _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__BEHAVIOR, exclude_packages)) {
    std::string content;
    std::string base_path;
    if (ament_index_cpp::get_resource(
          _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__BEHAVIOR, p, content, &base_path)) {
      for (const auto & line :
           auto_apms_util::splitString(content, _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_LINE_SEP)) {
        const std::vector<std::string> parts = auto_apms_util::splitString(
          line, _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_MARKER_FILE_FIELD_PER_LINE_SEP, false);
        if (parts.size() > 1) {
          BehaviorResourceIdentity i;
          i.category_name = parts[0];
          i.package_name = p;
          i.behavior_alias = parts[1];
          if (!include_categories.empty() && include_categories.find(i.category_name) == include_categories.end())
            continue;  // Skip identities not in the specified include categories
          identities.insert(i);
        }
      }
    }
  }
  return identities;
}

}  // namespace auto_apms_behavior_tree::core
