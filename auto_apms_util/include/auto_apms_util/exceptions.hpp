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

#include <stdexcept>

namespace auto_apms_util::exceptions
{

/**
 * @brief Base class for all exceptions of AutoAPMS
 */
class ExceptionBase : public std::runtime_error
{
public:
  explicit inline ExceptionBase(const std::string & msg) : std::runtime_error(msg) {}
};

struct ResourceError : public ExceptionBase
{
  using ExceptionBase::ExceptionBase;
};

struct ResourceIdentityFormatError : public ExceptionBase
{
  using ExceptionBase::ExceptionBase;
};

struct YAMLFormatError : public ExceptionBase
{
  using ExceptionBase::ExceptionBase;
};

}  // namespace auto_apms_util::exceptions
