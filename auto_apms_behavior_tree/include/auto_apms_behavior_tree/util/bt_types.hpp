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

#include "behaviortree_cpp/basic_types.h"

/// @cond
namespace BT
{

template <>
[[nodiscard]] std::vector<uint8_t> convertFromString<std::vector<uint8_t>>(StringView str);

template <>
[[nodiscard]] std::vector<bool> convertFromString<std::vector<bool>>(StringView str);

template <>
[[nodiscard]] std::vector<int64_t, std::allocator<int64_t>>
convertFromString<std::vector<int64_t, std::allocator<int64_t>>>(StringView str);

}  // namespace BT
/// @endcond
