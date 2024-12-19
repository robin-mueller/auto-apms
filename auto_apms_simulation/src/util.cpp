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

#include "auto_apms_simulation/util.hpp"

namespace auto_apms_simulation
{

std::string toStr(ExecutionResultMsg result)
{
  std::string status_str;
  switch (result.status) {
    case result.SUCCESS:
      status_str = "SUCCESS";
      break;
    case result.PRECONDITION_FAILURE:
      status_str = "PRECONDITION_FAILURE";
      break;
    case result.PLANNING_FAILURE:
      status_str = "PLANNING_FAILURE";
      break;
    case result.EXECUTION_FAILURE:
      status_str = "EXECUTION_FAILURE";
      break;
    case result.POSTCONDITION_FAILURE:
      status_str = "POSTCONDITION_FAILURE";
      break;
    case result.INVALID_ACTION:
      status_str = "INVALID_ACTION";
      break;
    case result.CANCELED:
      status_str = "CANCELED";
      break;
    default:
      status_str = "UNKOWN";
  }
  return result.message + " (Result status: " + status_str + ").";
}

}  // namespace auto_apms_simulation