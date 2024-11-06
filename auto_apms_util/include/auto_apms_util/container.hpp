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

#include <algorithm>
#include <vector>

namespace auto_apms_util
{

template <typename ValueT, typename AllocatorT, template <typename T, class A> class ContainerT>
bool contains(const ContainerT<ValueT, AllocatorT> & c, const ValueT & val)
{
  return std::find(c.begin(), c.end(), val) != c.end();
}

template <typename ContainerT>
ContainerT haveCommonElements(ContainerT c1, ContainerT c2)
{
  std::sort(c1.begin(), c1.end());
  std::sort(c2.begin(), c2.end());
  ContainerT intersection;
  std::set_intersection(c1.begin(), c1.end(), c2.begin(), c2.end(), std::back_inserter(intersection));
  return intersection;
}

}  // namespace auto_apms_util
