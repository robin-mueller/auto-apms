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

#include "auto_apms_behavior_tree_core/tree/script.hpp"

#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_util/string.hpp"
#include "behaviortree_cpp/scripting/script_parser.hpp"

namespace auto_apms_behavior_tree::core
{

Script::Script(std::vector<std::string> expressions) : expressions_(expressions)
{
  if (const BT::Result res = BT::ValidateScript(str()); !res) {
    throw exceptions::ScriptError(res.error());
  }
}

Script::Script(const std::string & str) : Script(std::vector<std::string>({str})) {}

Script::Script(const char * str) : Script(std::string(str)) {}

Script & Script::operator+=(const Script & rhs)
{
  for (const std::string & expr : rhs.expressions_) expressions_.push_back(expr);
  return *this;
}

std::string Script::str() const { return auto_apms_util::join(expressions_, "; "); }

Script operator+(Script lhs, const Script & rhs)
{
  lhs += rhs;  // reuse compound assignment
  return lhs;  // return the result by value (uses move constructor)
}

}  // namespace auto_apms_behavior_tree::core
