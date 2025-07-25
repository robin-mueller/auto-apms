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

#include "rclcpp/logger.hpp"

namespace auto_apms_util
{

/// @ingroup auto_apms_util
/// @{

/**
 * @brief Set the logging severity of a ROS 2 logger.
 * @param logger Logger instance.
 * @param severity Desired severity level encoded as a string. Must be one of DEBUG, INFO, WARN, ERROR, FATAL or UNSET.
 */
void setLoggingSeverity(const rclcpp::Logger & logger, const std::string & severity);

/**
 * @brief Set the logging severity of a ROS 2 logger.
 * @param logger Logger instance.
 * @param severity Desired severity level.
 */
void setLoggingSeverity(const rclcpp::Logger & logger, rclcpp::Logger::Level severity);

/// @}

}  // namespace auto_apms_util
