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

#include <string>

namespace auto_apms_core::util
{

enum class TextColor
{
  GREEN,
  RED,
  YELLOW,
  BLUE,
  MAGENTA,
  CYAN
};

/**
 * @brief Add ANSI color escape sequences to display the text in color when printed to console.
 *
 * The text color will be reset to default after the text ends.
 *
 * @ingroup auto_apms_core
 * @param text Text to be displayed.
 * @param color Desired color of the text.
 * @return String including corresponding ANSI color escape sequences.
 */
std::string ColoredText(const std::string& text, TextColor color);

}  // namespace auto_apms_core::util