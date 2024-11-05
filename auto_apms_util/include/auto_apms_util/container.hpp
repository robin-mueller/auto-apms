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

#include <vector>
#include <algorithm>

namespace auto_apms_util
{

template <typename ContainerT>
ContainerT haveCommonElements(ContainerT vec1, ContainerT vec2)
{
  std::sort(vec1.begin(), vec1.end());
  std::sort(vec2.begin(), vec2.end());
  ContainerT intersection;
  std::set_intersection(vec1.begin(), vec1.end(), vec2.begin(), vec2.end(), std::back_inserter(intersection));
  return intersection;
}

}  // namespace auto_apms_util
