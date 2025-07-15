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
  if (std::size_t pos = identity.find(RESOURCE_IDENTITY_CATEGORY_SEPARATOR); pos == std::string::npos) {
    category_name = "";
    resource_part = identity;
  } else {
    category_name = identity.substr(0, pos);
    resource_part = identity.substr(pos + RESOURCE_IDENTITY_CATEGORY_SEPARATOR.size());
  }
  if (resource_part.empty()) {
    throw auto_apms_util::exceptions::ResourceIdentityFormatError(
      "Behavior resource identity string '" + identity + "' is invalid: Package and resource name must not be empty.");
  }
  if (std::size_t pos = resource_part.find(RESOURCE_IDENTITY_RESOURCE_SEPARATOR); pos == std::string::npos) {
    // If only a single token is given, assume it's resource_name
    package_name = "";
    resource_name = resource_part;
  } else {
    package_name = resource_part.substr(0, pos);
    resource_name = resource_part.substr(pos + RESOURCE_IDENTITY_RESOURCE_SEPARATOR.size());
  }
  if (package_name.empty() && resource_name.empty()) {
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
    str += category_name + RESOURCE_IDENTITY_CATEGORY_SEPARATOR;
  }
  str += package_name + RESOURCE_IDENTITY_RESOURCE_SEPARATOR + resource_name;
  return str;
}

bool BehaviorResourceIdentity::empty() const { return package_name.empty() && resource_name.empty(); }

}  // namespace auto_apms_behavior_tree::core
