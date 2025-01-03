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

#include "auto_apms_behavior_tree_core/node/node_registration_options.hpp"

namespace auto_apms_behavior_tree::core
{

// clang-format off
const std::string NodeRegistrationOptions::PARAM_NAME_CLASS = _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_CLASS;
const std::string NodeRegistrationOptions::PARAM_NAME_PORT = _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_PORT;
const std::string NodeRegistrationOptions::PARAM_NAME_REQUEST_TIMEOUT = _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_REQUEST_TIMEOUT;
const std::string NodeRegistrationOptions::PARAM_NAME_WAIT_TIMEOUT = _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_WAIT_TIMEOUT;
const std::string NodeRegistrationOptions::PARAM_NAME_ALLOW_UNREACHABLE = _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_ALLOW_UNREACHABLE;
const std::string NodeRegistrationOptions::PARAM_NAME_LOGGER_LEVEL = _AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_PARAM_LOGGER_LEVEL;
// clang-format on

bool NodeRegistrationOptions::valid() const { return !class_name.empty(); }

}  // namespace auto_apms_behavior_tree::core