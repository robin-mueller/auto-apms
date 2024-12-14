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

#include "auto_apms_util/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rcutils/error_handling.h"
#include "rcutils/logging.h"

namespace auto_apms_util
{

void exposeToGlobalDebugLogging(const rclcpp::Logger & logger)
{
#ifdef _AUTO_APMS_DEBUG_LOGGING
  auto ret = rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK) {
    const std::string msg = rcutils_get_error_string().str;
    rcutils_reset_error();
    throw exceptions::SetLoggingSeverityError("Failed to expose the logger to global DEBUG logging: " + msg);
  }
#endif
}

void setLoggingSeverity(const rclcpp::Logger & logger, rclcpp::Logger::Level severity_level)
{
  switch (severity_level) {
    case rclcpp::Logger::Level::Debug:
      setLoggingSeverity(logger, "DEBUG");
      break;
    case rclcpp::Logger::Level::Info:
      setLoggingSeverity(logger, "INFO");
      break;
    case rclcpp::Logger::Level::Warn:
      setLoggingSeverity(logger, "WARN");
      break;
    case rclcpp::Logger::Level::Error:
      setLoggingSeverity(logger, "ERROR");
      break;
    case rclcpp::Logger::Level::Fatal:
      setLoggingSeverity(logger, "FATAL");
      break;
    case rclcpp::Logger::Level::Unset:
      setLoggingSeverity(logger, "UNSET");
      break;
  }
}

void setLoggingSeverity(const rclcpp::Logger & logger, const std::string & severity_string)
{
  int severity;
  rcutils_ret_t ret =
    rcutils_logging_severity_level_from_string(severity_string.c_str(), rcl_get_default_allocator(), &severity);
  if (RCUTILS_RET_LOGGING_SEVERITY_STRING_INVALID == ret) {
    throw exceptions::SetLoggingSeverityError("Unknown severity string '" + severity_string + "'");
  }
  if (ret != RCUTILS_RET_OK) {
    const std::string msg = rcutils_get_error_string().str;
    rcutils_reset_error();
    throw exceptions::SetLoggingSeverityError(msg);
  }

  ret = rcutils_logging_set_logger_level(logger.get_name(), severity);
  if (ret != RCUTILS_RET_OK) {
    const std::string msg = rcutils_get_error_string().str;
    rcutils_reset_error();
    throw exceptions::SetLoggingSeverityError(msg);
  }
}

}  // namespace auto_apms_util
