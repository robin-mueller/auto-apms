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
#include <set>
#include <vector>

namespace auto_apms_util
{

/// @ingroup auto_apms_util
/// @{

/**
 * @brief Check whether a particular container structure contains a value.
 * @tparam ValueT Type of the values inside the container.
 * @tparam AllocatorT Container allocator type.
 * @param c Container to be searched.
 * @param val Comparison value.
 * @return `true` if @p c contains @p val, `false` otherwise.
 */
template <typename ValueT, typename AllocatorT, template <typename T, class A> class ContainerT>
bool contains(const ContainerT<ValueT, AllocatorT> & c, const ValueT & val)
{
  for (const ValueT & v : c)
    if (v == val) return true;
  return false;
}

/**
 * @brief Assemble common elements of two sets.
 * @tparam KeyT Type of the keys within the set.
 * @tparam CompareT Comparator type.
 * @tparam AllocatorT Allocator type.
 * @param c1 First set.
 * @param c2 Second set.
 * @return Set of common elements present in @p c1 as well as @p c2.
 */
template <typename KeyT, typename CompareT, typename AllocatorT>
std::set<KeyT, CompareT, AllocatorT> getCommonElements(
  std::set<KeyT, CompareT, AllocatorT> c1, std::set<KeyT, CompareT, AllocatorT> c2)
{
  std::set<KeyT, CompareT, AllocatorT> intersect;
  std::set_intersection(c1.begin(), c1.end(), c2.begin(), c2.end(), std::inserter(intersect, intersect.begin()));
  return intersect;
}

/**
 * @brief Assemble common elements of two vectors.
 * @tparam KeyT Type of the values within the vector.
 * @tparam AllocatorT Allocator type.
 * @param c1 First vector.
 * @param c2 Second vector.
 * @return Vector of common elements present in @p c1 as well as @p c2.
 */
template <typename KeyT, typename AllocatorT>
std::vector<KeyT, AllocatorT> getCommonElements(std::vector<KeyT, AllocatorT> c1, std::vector<KeyT, AllocatorT> c2)
{
  std::sort(c1.begin(), c1.end());
  std::sort(c2.begin(), c2.end());
  std::vector<KeyT, AllocatorT> intersect;
  std::set_intersection(c1.begin(), c1.end(), c2.begin(), c2.end(), std::back_inserter(intersect));
  return intersect;
}

/// @}

}  // namespace auto_apms_util
