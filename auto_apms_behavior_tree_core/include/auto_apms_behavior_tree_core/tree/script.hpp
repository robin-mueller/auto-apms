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

#include <string>
#include <vector>

namespace auto_apms_behavior_tree::core
{

/**
 * @brief Class that encapsulates behavior tree script expressions.
 *
 * Users may create a script expression by passing a string. More expression can be added using the plus operator. Each
 * expressions is validated at the time of construction.
 *
 * @sa https://www.behaviortree.dev/docs/guides/scripting
 */
class Script
{
  friend class TreeDocument;

public:
  /**
   * @brief Create an empty script.
   */
  Script() = default;

  /**
   * @brief Create a script with multiple expressions.
   * @param expressions Vector of expression strings.
   * @throw auto_apms_behavior_tree::exceptions::ScriptError if script validation fails.
   */
  Script(std::vector<std::string> expressions);

  /**
   * @brief Create a script with a single expression.
   * @param str Expression string.
   * @throw auto_apms_behavior_tree::exceptions::ScriptError if script validation fails.
   */
  Script(const std::string & str);

  /**
   * @brief Create a script with a single expression.
   * @param str C-style expression string.
   * @throw auto_apms_behavior_tree::exceptions::ScriptError if script validation fails.
   */
  Script(const char * str);

  Script & operator+=(const Script & rhs);

  friend Script operator+(Script lhs, const Script & rhs);

  /**
   * @brief Concatenate all expressions of this instance to a single string.
   * @return String representing the script.
   */
  std::string str() const;

private:
  std::vector<std::string> expressions_;
};

}  // namespace auto_apms_behavior_tree::core