
.. _program_listing_file_include_auto_apms_util_string.hpp:

Program Listing for File string.hpp
===================================

|exhale_lsh| :ref:`Return to documentation for file <file_include_auto_apms_util_string.hpp>` (``include/auto_apms_util/string.hpp``)

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
   
   #include <string.h>
   
   #include <map>
   #include <vector>
   
   // Additionally include some string utils from rcpputils
   #include "rcpputils/join.hpp"
   
   namespace auto_apms_util
   {
   
   enum class TextColor
   {
     GREEN,
     RED,
     YELLOW,
     BLUE,
     MAGENTA,
     CYAN
   };
   
   using rcpputils::join;
   
   std::vector<std::string> splitString(const std::string & str, const std::string & delimiter, bool remove_empty = true);
   
   std::string printMap(
     const std::map<std::string, std::string> & map, const std::string & key_val_sep = "=",
     const std::string & entry_sep = ", ");
   
   std::string makeColoredText(const std::string & text, TextColor color);
   
   std::string trimWhitespaces(const std::string & str);
   
   std::string toCamelCase(const std::string & str);
   
   std::string toSnakeCase(const std::string & str);
   
   
   }  // namespace auto_apms_util
