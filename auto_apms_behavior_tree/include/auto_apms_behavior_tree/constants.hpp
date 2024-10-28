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

/**
 * @defgroup auto_apms_behavior_tree AutoAPMS - Behavior Tree
 * @brief Useful tooling for Behavior Tree development.
 */

namespace auto_apms_behavior_tree
{

// BT Executor definitions
const char BT_EXECUTOR_UPLOAD_TREE_SERVICE_NAME_SUFFIX[] = "/upload";
const char BT_EXECUTOR_RUN_ACTION_NAME_SUFFIX[] = "/run";
const char BT_EXECUTOR_COMMAND_ACTION_NAME_SUFFIX[] = "/command";

}  // namespace auto_apms_behavior_tree
