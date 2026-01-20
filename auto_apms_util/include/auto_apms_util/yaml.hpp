// Copyright 2024 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <algorithm>
#include <map>
#include <string>

#include "auto_apms_util/exceptions.hpp"
#include "auto_apms_util/filesystem.hpp"
#include "yaml-cpp/yaml.h"

/// @ingroup auto_apms_util
/// @{

/**
 * @brief Macro for defining YAML encode/decode methods for a class.
 * @param ClassType Fully qualified name of the class.
 */
#define AUTO_APMS_UTIL_DEFINE_YAML_CONVERSION_METHODS(ClassType)                                             \
  static ClassType fromFile(const std::string & path)                                                        \
  {                                                                                                          \
    if (auto_apms_util::isFileEmpty(path)) return ClassType();                                               \
    try {                                                                                                    \
      return YAML::LoadFile(path).as<ClassType>();                                                           \
    } catch (const YAML::ParserException & e) {                                                              \
      throw auto_apms_util::exceptions::YAMLFormatError(                                                     \
        "Format error when loading YAML file " + path + ": " + std::string(e.what()));                       \
    }                                                                                                        \
  }                                                                                                          \
  static ClassType decode(const std::string & str)                                                           \
  {                                                                                                          \
    const bool empty = std::all_of(str.begin(), str.end(), [](unsigned char c) { return std::isspace(c); }); \
    return empty ? ClassType() : YAML::Load(str).as<ClassType>();                                            \
  }                                                                                                          \
  std::string encode() const                                                                                 \
  {                                                                                                          \
    YAML::Node root;                                                                                         \
    root = *this;                                                                                            \
    YAML::Emitter out;                                                                                       \
    out << root;                                                                                             \
    if (!out.good()) {                                                                                       \
      throw std::runtime_error("Error trying to encode to YAML: " + out.GetLastError());                     \
    }                                                                                                        \
    return out.c_str();                                                                                      \
  }

/// @}
