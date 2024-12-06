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
#include <iterator>
#include <sstream>

namespace auto_apms_util
{

std::vector<std::string> splitString(const std::string & str, const std::string & delimiter, bool remove_empty)
{
  std::vector<std::string> parts;
  size_t start = 0;
  size_t end = str.find(delimiter);
  while (end != std::string::npos) {
    parts.push_back(str.substr(start, end - start));  // Add the part before the delimiter
    start = end + delimiter.length();                 // Move start to after the delimiter
    end = str.find(delimiter, start);                 // Find the next occurrence of the delimiter
  }
  // Add the last part after the last delimiter (or the entire string if no delimiter was found)
  parts.push_back(str.substr(start));

  // Remove empty strings if desired
  if (remove_empty) {
    auto it = parts.begin();
    while (it != parts.end()) {
      if (*it == "")
        it = parts.erase(it);
      else
        ++it;
    }
  }
  return parts;
}

std::string printMap(
  const std::map<std::string, std::string> & map, const std::string & key_val_sep, const std::string & entry_sep)
{
  std::ostringstream oss;
  bool first = true;
  for (const auto & [key, val] : map) {
    if (!first) {
      oss << entry_sep;  // Separator between entries
    }
    oss << key << key_val_sep << val;  // Separator between key and value
    first = false;
  }
  return oss.str();
}

std::string makeColoredText(const std::string & text, TextColor color)
{
  switch (color) {
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

std::string trimWhitespaces(const std::string & str)
{
  std::string::const_iterator start = str.begin();
  while (start != str.end() && std::isspace(*start)) {
    ++start;
  }
  std::string::const_iterator end = str.end();
  do {
    --end;
  } while (std::distance(start, end) > 0 && std::isspace(*end));
  return std::string(start, end + 1);
}

}  // namespace auto_apms_util
