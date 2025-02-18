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

#include "auto_apms_util/exceptions.hpp"

namespace auto_apms_behavior_tree::exceptions
{

struct NodeManifestError : public auto_apms_util::exceptions::ExceptionBase
{
  using ExceptionBase::ExceptionBase;
};

struct RosNodeError : public auto_apms_util::exceptions::ExceptionBase
{
  using ExceptionBase::ExceptionBase;
};

struct TreeDocumentError : public auto_apms_util::exceptions::ExceptionBase
{
  using ExceptionBase::ExceptionBase;
};

struct ScriptError : public auto_apms_util::exceptions::ExceptionBase
{
  using ExceptionBase::ExceptionBase;
};

struct NodeRegistrationError : public auto_apms_util::exceptions::ExceptionBase
{
  using ExceptionBase::ExceptionBase;
};

struct TreeBuildError : public auto_apms_util::exceptions::ExceptionBase
{
  using ExceptionBase::ExceptionBase;
};

}  // namespace auto_apms_behavior_tree::exceptions
