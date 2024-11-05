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

#include "auto_apms_util/string.hpp"

#include <algorithm>
#include <sstream>
#include <iterator>

namespace auto_apms_util
{

std::vector<std::string> splitString(const std::string& str, const std::string& delimiter, bool preserve_empty)
{
  std::vector<std::string> parts;
  size_t start = 0;
  size_t end = str.find(delimiter);
  while (end != std::string::npos)
  {
    parts.push_back(str.substr(start, end - start));  // Add the part before the delimiter
    start = end + delimiter.length();                 // Move start to after the delimiter
    end = str.find(delimiter, start);                 // Find the next occurrence of the delimiter
  }
  // Add the last part after the last delimiter (or the entire string if no delimiter was found)
  parts.push_back(str.substr(start));

  // Remove empty strings if desired
  if (!preserve_empty)
    parts.erase(std::remove(parts.begin(), parts.end(), ""), parts.end());

  return parts;
}

std::string makeColoredText(const std::string& text, TextColor color)
{
  switch (color)
  {
    case TextColor::GREEN:
      return "\x1b[32m" + text + "\x1b[0m";
    case TextColor::RED:
      return "\x1b[31m" + text + "\x1b[0m";
    case TextColor::YELLOW:
      return "\x1b[33m" + text + "\x1b[0m";
    case TextColor::BLUE:
      return "\x1b[34m" + text + "\x1b[0m";
    case TextColor::MAGENTA:
      return "\x1b[35m" + text + "\x1b[0m";
    case TextColor::CYAN:
      return "\x1b[36m" + text + "\x1b[0m";
  }
  return text;
}

}  // namespace auto_apms_util