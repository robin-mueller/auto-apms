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

namespace auto_apms_util
{

/// @ingroup auto_apms_util
/// @{

/**
 * @brief Determine if a file is empty.
 *
 * A file is considered empty if it has no content or the content consists of whitespace characters only. If there is
 * any non-whitespace character, this function returns `false`.
 *
 * @param path Path to the file.
 * @return `true` if the file has no content or consists of whitespaces only, `false` otherwise.
 */
bool isFileEmpty(const std::string & path);

/// @}

}  // namespace auto_apms_util