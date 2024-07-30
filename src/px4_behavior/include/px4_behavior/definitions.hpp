#pragma once

namespace px4_behavior {

// Task names (ROS 2 actions)
const char ARM_DISARM_TASK_NAME[] = "arm_disarm";
const char ENABLE_HOLD_TASK_NAME[] = "enable_hold";
const char GO_TO_TASK_NAME[] = "go_to";
const char LAND_TASK_NAME[] = "land";
const char RTL_TASK_NAME[] = "rtl";
const char TAKEOFF_TASK_NAME[] = "takeoff";
const char MISSION_TASK_NAME[] = "mission";

const char BT_EXECUTOR_UPLOAD_SERVICE_NAME_SUFFIX[] = "/upload";
const char BT_EXECUTOR_LAUNCH_ACTION_NAME_SUFFIX[] = "/run";
const char BT_EXECUTOR_COMMAND_ACTION_NAME_SUFFIX[] = "/command";

}  // namespace px4_behavior
