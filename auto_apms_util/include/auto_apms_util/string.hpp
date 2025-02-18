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

#include <string.h>

#include <map>
#include <vector>

// Additionally include some string utils from rcpputils
#include "rcpputils/join.hpp"

namespace auto_apms_util
{
/// @ingroup auto_apms_util
/// @{

enum class TextColor
{
  GREEN,
  RED,
  YELLOW,
  BLUE,
  MAGENTA,
  CYAN
};

using rcpputils::join;

/**
 * @brief Split a string into multiple tokens using a specific delimiter string (Delimiter may consist of multiple
 * characters).
 *
 * Will remove empty strings if @p remove_empty is `true` (Default). So for example with the delimiter being
 * `::` passing the string `::foo` will output a vector with one elements {"foo"}. If you want {"", "foo"}, you must set
 * @p remove_empty to `false`.
 *
 * @param[in] str String to split into multiple tokens.
 * @param[in] delimiter Delimiter string at which the string shall be split.
 * @param[in] remove_empty Remove empty string tokens in the result vector.
 * @return Vector of string representing the string's tokens without the delimiter.
 */
std::vector<std::string> splitString(const std::string & str, const std::string & delimiter, bool remove_empty = true);

/**
 * @brief Converts a map to a string representation that is suited for printing to console.
 * @param map Map to be converted to string.
 * @param key_val_sep Separator for the key-value pairs.
 * @param entry_sep Separator for the map's entries.
 * @return String that encodes @p map.
 */
std::string printMap(
  const std::map<std::string, std::string> & map, const std::string & key_val_sep = "=",
  const std::string & entry_sep = ", ");

/**
 * @brief Add ANSI color escape sequences to display the text in color when printed to console.
 *
 * The text color will be reset to default after the text ends.
 *
 * @ingroup auto_apms_util
 * @param text Text to be displayed.
 * @param color Desired color of the text.
 * @return String including corresponding ANSI color escape sequences.
 */
std::string makeColoredText(const std::string & text, TextColor color);

/**
 * @brief Trim whitespaces from both ends of a string
 * @param str String to trim.
 * @return Trimmed string.
 */
std::string trimWhitespaces(const std::string & str);

/**
 * @brief Transform a string to camelCase.
 * @param[in] str String to transform.
 * @return Transformed string.
 */
std::string toCamelCase(const std::string & str);

/**
 * @brief Transform a string to snake_case.
 * @param[in] str String to transform.
 * @return Transformed string.
 */
std::string toSnakeCase(const std::string & str);

/// @}

}  // namespace auto_apms_util
