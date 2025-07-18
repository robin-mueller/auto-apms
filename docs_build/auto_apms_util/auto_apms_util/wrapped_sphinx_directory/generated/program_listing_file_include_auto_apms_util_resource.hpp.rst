
.. _program_listing_file_include_auto_apms_util_resource.hpp:

Program Listing for File resource.hpp
=====================================

|exhale_lsh| :ref:`Return to documentation for file <file_include_auto_apms_util_resource.hpp>` (``include/auto_apms_util/resource.hpp``)

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
   
   #include <map>
   #include <set>
   #include <string>
   #include <type_traits>
   #include <vector>
   
   #include "auto_apms_util/container.hpp"
   #include "auto_apms_util/exceptions.hpp"
   #include "auto_apms_util/string.hpp"
   #include "pluginlib/class_loader.hpp"
   
   namespace auto_apms_util
   {
   
   
   std::set<std::string> getPackagesWithResourceType(
     const std::string & resource_type, const std::set<std::string> & exclude_packages = {});
   
   std::set<std::string> getPackagesWithPluginResources(const std::set<std::string> & exclude_packages = {});
   
   std::string getPluginXMLPath(const std::string & package);
   
   std::vector<std::string> collectPluginXMLPaths(const std::set<std::string> & exclude_packages = {});
   
   
   // clang-format off
   // clang-format on
   template <typename BaseT>
   class PluginClassLoader : public pluginlib::ClassLoader<BaseT>
   {
   public:
     PluginClassLoader(
       const std::string & base_package, const std::string & base_class,
       const std::set<std::string> & exclude_packages = {});
   
     static PluginClassLoader makeUnambiguousPluginClassLoader(
       const std::string & base_package, const std::string & base_class,
       const std::set<std::string> & exclude_packages = {},
       const std::map<std::string, std::string> & reserved_names = {});
   
     std::map<std::string, std::string> getClassPackageMap();
   };
   
   // #####################################################################################################################
   // ################################              DEFINITIONS              ##############################################
   // #####################################################################################################################
   
   template <typename BaseT>
   inline PluginClassLoader<BaseT>::PluginClassLoader(
     const std::string & base_package, const std::string & base_class, const std::set<std::string> & exclude_packages)
   : pluginlib::ClassLoader<BaseT>(base_package, base_class, "", collectPluginXMLPaths(exclude_packages))
   {
   }
   
   template <typename BaseT>
   inline PluginClassLoader<BaseT> PluginClassLoader<BaseT>::makeUnambiguousPluginClassLoader(
     const std::string & base_package, const std::string & base_class, const std::set<std::string> & exclude_packages,
     const std::map<std::string, std::string> & reserved_names)
   {
     std::map<std::string, std::vector<std::string>> packages_for_class_name;
     const std::set<std::string> packages =
       getPackagesWithResourceType(_AUTO_APMS_UTIL__RESOURCE_TYPE_NAME__PLUGINLIB, exclude_packages);
     for (const auto & package : packages) {
       auto single_package_loader =
         pluginlib::ClassLoader<BaseT>(base_package, base_class, "", {getPluginXMLPath(package)});
       for (const auto & class_name : single_package_loader.getDeclaredClasses()) {
         packages_for_class_name[class_name].push_back(package);
       }
     }
   
     // Reserved class names are considered as declared
     for (const auto & [class_name, package] : reserved_names) {
       packages_for_class_name[class_name].push_back(package + "(Build package)");
     }
   
     // Determine if there are duplicate class names
     std::vector<std::string> error_details;
     for (const auto & [class_name, packages] : packages_for_class_name) {
       if (packages.size() > 1) {
         error_details.push_back(
           "- Class '" + class_name + "' found in packages ['" + auto_apms_util::join(packages, "', '") + "'].");
       }
     }
     if (!error_details.empty()) {
       throw exceptions::ResourceError(
         "Ambiguous class names found! PluginClassLoader (Base: '" + base_class +
         "') created with makeUnambiguousPluginClassLoader() won't register resources from packages "
         "that use already existing lookup names. Found the following duplicates:\n" +
         auto_apms_util::join(error_details, "\n"));
     }
     return {base_package, base_class, exclude_packages};
   }
   
   template <typename BaseT>
   inline std::map<std::string, std::string> PluginClassLoader<BaseT>::getClassPackageMap()
   {
     std::map<std::string, std::string> map;
     for (const std::string & class_name : this->getDeclaredClasses()) {
       map[class_name] = this->getClassPackage(class_name);
     }
     return map;
   }
   
   }  // namespace auto_apms_util
