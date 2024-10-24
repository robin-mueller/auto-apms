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

#include "auto_apms_core/util/console.hpp"

namespace auto_apms_core::util {

std::string ColoredText(const std::string& text, TextColor color)
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

}  // namespace auto_apms_core::util