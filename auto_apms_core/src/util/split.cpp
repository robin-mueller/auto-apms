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

#include "auto_apms_core/util/split.hpp"

#include <algorithm>

namespace auto_apms_core::util {

std::vector<std::string> SplitString(const std::string& str, const std::string& delimiter, bool preserve_empty)
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
    if (!preserve_empty) parts.erase(std::remove(parts.begin(), parts.end(), ""), parts.end());

    return parts;
}

}  // namespace auto_apms_core::util
