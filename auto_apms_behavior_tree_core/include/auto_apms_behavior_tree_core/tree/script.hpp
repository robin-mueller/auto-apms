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

#include <string>
#include <vector>

namespace auto_apms_behavior_tree::core
{

class Script
{
  friend class TreeDocument;

public:
  Script() = default;

  Script(std::vector<std::string> expressions);

  Script(const std::string & str);

  Script(const char * str);

  Script & operator+=(const Script & rhs);

  friend Script operator+(Script lhs, const Script & rhs);

  std::string str() const;

private:
  std::vector<std::string> expressions_;
};

}  // namespace auto_apms_behavior_tree::core