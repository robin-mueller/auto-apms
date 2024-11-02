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

#include <filesystem>
#include <iostream>
#include <set>

#include "auto_apms_core/exceptions.hpp"
#include "auto_apms_core/resources.hpp"
#include "auto_apms_core/util/string.hpp"
#include "auto_apms_behavior_tree/node/node_manifest.hpp"
#include "auto_apms_behavior_tree/resource/node_class_loader.hpp"

using namespace auto_apms_behavior_tree;

int main(int argc, char** argv)
{
  if (argc < 5)
  {
    std::cerr << "create_node_manifest: Missing inputs! The program requires: \n\t1.) the yaml "
                 "node manifest files (separated by ';').\n\t2.) Build information for nodes supposed to be "
                 "registered during build time (List of '<class_name>@<library_build_path>' "
                 "separated by ';').\n\t3.) The name of the package that provides the build targets.\n\t4.) Output "
                 "file for the complete node plugin manifest.\n\t";
    std::cerr << "Usage: create_node_manifest <manifest_files> <build_infos> <build_package_name> "
                 "<output_file>\n";
    return EXIT_FAILURE;
  }

  try
  {
    std::vector<std::string> manifest_files;
    for (const auto& path : auto_apms_core::util::splitString(argv[1], ";"))
    {
      manifest_files.push_back(std::filesystem::absolute(path).string());
    }
    const std::vector<std::string> build_infos = auto_apms_core::util::splitString(argv[2], ";");
    const std::string build_package_name = argv[3];
    const std::filesystem::path output_file{ std::filesystem::absolute(argv[4]) };

    // Ensure that arguments are not empty
    if (manifest_files.empty())
    {
      throw std::runtime_error("Argument manifest_files must not be empty");
    }
    if (output_file.empty())
    {
      throw std::runtime_error("Argument output_file must not be empty.");
    }

    // Ensure correct extensions
    if (output_file.extension() != ".yaml")
    {
      throw std::runtime_error("Output file '" + output_file.string() + "' has wrong extension. Must be '.yaml'.");
    }

    // Retrieve plugin library paths from build info
    std::map<std::string, std::string> build_lib_paths;
    for (const auto& build_info : build_infos)
    {
      std::vector<std::string> parts = auto_apms_core::util::splitString(build_info, "@");
      if (parts.size() != 2)
      {
        throw std::runtime_error("Invalid build info entry ('" + build_info + "').");
      }
      const std::string& class_name = parts[0];
      const std::string& build_path = parts[1];
      if (build_lib_paths.find(class_name) != build_lib_paths.end())
      {
        throw std::runtime_error("Node class '" + class_name + "' is specified multiple times in build infos.");
      }
      build_lib_paths[class_name] = build_path;  // {class_name: build_path}
    }

    auto output_manifest = NodeManifest::fromFiles(manifest_files);
    auto all_but_build_package =
        auto_apms_core::getAllPackagesWithResource(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__NODE);
    all_but_build_package.erase(build_package_name);

    /**
     * Trying to construct NodePluginClassLoader will work completely fine during build time for all packages EXCEPT the
     * original auto_apms_behavior_tree package. Will throw pluginlib::ClassLoaderException in this case, because the
     * pluginlib::ClassLoader constructor initially checks wether the package containing the base class is installed.
     * Therefore we MUST avoid triggering this script during build time of auto_apms_behavior_tree.
     */
    NodePluginClassLoader loader(all_but_build_package);
    for (const auto& [node_name, params] : output_manifest.getInternalMap())
    {
      try
      {
        output_manifest[node_name] = NodeManifest({ { node_name, params } }).autoComplete(loader)[node_name];
      }
      catch (const auto_apms_core::exceptions::ResourceNotFoundError& e)
      {
        if (build_lib_paths.find(params.class_name) == build_lib_paths.end())
        {
          throw std::runtime_error("Node plugin '" + node_name + "' ('" + params.class_name +
                                   "') cannot be found. It doesn't exist in any installed package and is "
                                   "also not being built by the current one.");
        }
        // Store the temporary library path to be used during build time until the install is available
        output_manifest[node_name].library = build_lib_paths[params.class_name];
        output_manifest[node_name].package = build_package_name;
      }
    }

    // Save the manifest
    output_manifest.toFile(output_file);

    // Print unique list of libraries to stdout
    std::set<std::string> paths;
    for (const auto& [node_name, params] : output_manifest.getInternalMap())
    {
      const auto& path = params.library;
      if (const auto& [_, success] = paths.insert(path); success)
      {
        std::cout << path << ';';
      }
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "ERROR (create_node_manifest): " << e.what() << "\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
