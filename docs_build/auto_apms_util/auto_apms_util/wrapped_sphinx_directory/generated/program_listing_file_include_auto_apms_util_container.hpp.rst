
.. _program_listing_file_include_auto_apms_util_container.hpp:

Program Listing for File container.hpp
======================================

|exhale_lsh| :ref:`Return to documentation for file <file_include_auto_apms_util_container.hpp>` (``include/auto_apms_util/container.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

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
   #include <set>
   #include <vector>
   
   namespace auto_apms_util
   {
   
   
   template <typename ValueT, typename AllocatorT, template <typename T, class A> class ContainerT>
   bool contains(const ContainerT<ValueT, AllocatorT> & c, const ValueT & val)
   {
     for (const ValueT & v : c)
       if (v == val) return true;
     return false;
   }
   
   template <typename KeyT, typename CompareT, typename AllocatorT>
   std::set<KeyT, CompareT, AllocatorT> getCommonElements(
     std::set<KeyT, CompareT, AllocatorT> c1, std::set<KeyT, CompareT, AllocatorT> c2)
   {
     std::set<KeyT, CompareT, AllocatorT> intersect;
     std::set_intersection(c1.begin(), c1.end(), c2.begin(), c2.end(), std::inserter(intersect, intersect.begin()));
     return intersect;
   }
   
   template <typename KeyT, typename AllocatorT>
   std::vector<KeyT, AllocatorT> getCommonElements(std::vector<KeyT, AllocatorT> c1, std::vector<KeyT, AllocatorT> c2)
   {
     std::sort(c1.begin(), c1.end());
     std::sort(c2.begin(), c2.end());
     std::vector<KeyT, AllocatorT> intersect;
     std::set_intersection(c1.begin(), c1.end(), c2.begin(), c2.end(), std::back_inserter(intersect));
     return intersect;
   }
   
   
   }  // namespace auto_apms_util
