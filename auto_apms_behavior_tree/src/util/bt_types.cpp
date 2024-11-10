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

#include "auto_apms_behavior_tree/util/bt_types.hpp"

/// @cond
namespace BT
{

template <>
std::vector<uint8_t> convertFromString<std::vector<uint8_t>>(StringView str)
{
  auto parts = splitString(str, ';');
  std::vector<uint8_t> output;
  output.reserve(parts.size());
  for (const StringView & part : parts) {
    output.push_back(convertFromString<uint8_t>(part));
  }
  return output;
}

template <>
std::vector<bool> convertFromString<std::vector<bool>>(StringView str)
{
  auto parts = splitString(str, ';');
  std::vector<bool> output;
  output.reserve(parts.size());
  for (const StringView & part : parts) {
    output.push_back(convertFromString<bool>(part));
  }
  return output;
}

template <>
[[nodiscard]] std::vector<int64_t, std::allocator<int64_t>>
convertFromString<std::vector<int64_t, std::allocator<int64_t>>>(StringView str)
{
  auto parts = splitString(str, ';');
  std::vector<int64_t, std::allocator<int64_t>> output;
  output.reserve(parts.size());
  for (const StringView & part : parts) {
    output.push_back(convertFromString<int64_t>(part));
  }
  return output;
}

}  // namespace BT
/// @endcond