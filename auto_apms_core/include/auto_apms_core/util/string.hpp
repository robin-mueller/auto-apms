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
#include <vector>

namespace auto_apms_core::util
{

/**
 * @brief Split a string into multiple tokens using a specific delimiter string (Delimiter may consist of multiple
 * characters).
 *
 * Will preserve empty strings if @p preserve_empty is `true` (Default), so for example with the delimiter being
 * `::` passing the string `::foo` will output a vector with two elements {"", "foo"}.
 *
 * @ingroup auto_apms_core
 * @param str String to split into multiple tokens.
 * @param delimiter Delimiter string at which the string shall be split.
 * @param preserve_empty Preserve empty string tokens in the result vector.
 * @return Vector of string representing the string's tokens without the delimiter.
 */
std::vector<std::string> splitString(const std::string& str, const std::string& delimiter, bool preserve_empty = true);

}  // namespace auto_apms_core::util
