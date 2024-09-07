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

#include "px4_behavior/action_client.hpp"
#include "px4_behavior/bt_executor.hpp"
#include "px4_behavior/bt_executor_client.hpp"
#include "px4_behavior/bt_factory.hpp"
#include "px4_behavior/bt_ros2_node.hpp"
#include "px4_behavior/get_resource.hpp"
#include "px4_behavior/mode.hpp"
#include "px4_behavior/mode_executor.hpp"
#include "px4_behavior/register_behavior_tree_node_macro.hpp"
#include "px4_behavior/task_base.hpp"

namespace px4_behavior {

// Task names (ROS 2 actions)
const char ARM_DISARM_TASK_NAME[] = "arm_disarm";
const char ENABLE_HOLD_TASK_NAME[] = "enable_hold";
const char GO_TO_TASK_NAME[] = "go_to";
const char LAND_TASK_NAME[] = "land";
const char RTL_TASK_NAME[] = "rtl";
const char TAKEOFF_TASK_NAME[] = "takeoff";
const char MISSION_TASK_NAME[] = "mission";

// BT Executor definitions
const char BT_EXECUTOR_UPLOAD_SERVICE_NAME_SUFFIX[] = "/upload";
const char BT_EXECUTOR_LAUNCH_ACTION_NAME_SUFFIX[] = "/run";
const char BT_EXECUTOR_COMMAND_ACTION_NAME_SUFFIX[] = "/command";

}  // namespace px4_behavior
