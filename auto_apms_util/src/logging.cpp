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

#include "auto_apms_util/logging.hpp"

#include "rclcpp/logging.hpp"

namespace auto_apms_util
{

void exposeToDebugLogging(const rclcpp::Logger& logger)
{
#ifdef _AUTO_APMS_DEBUG_LOGGING
  auto ret = rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK)
  {
    RCLCPP_ERROR(logger, "Error setting severity: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }
#endif
}

}  // namespace auto_apms_util
