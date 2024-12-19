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

#include "auto_apms_util/filesystem.hpp"

#include <cctype>
#include <exception>
#include <fstream>

namespace auto_apms_util
{

bool isFileEmpty(const std::string & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Couldn't open output file'" + path + "' to check if it is empty.");
  }
  // Read the file character by character
  char ch;
  while (file.get(ch)) {
    if (!std::isspace(static_cast<unsigned char>(ch))) {
      // Found a non-whitespace character
      return false;
    }
  }
  return true;
}

}  // namespace auto_apms_util