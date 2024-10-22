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

#include "auto_apms_behavior_tree/exceptions.hpp"
#include "auto_apms_behavior_tree/node/plugin_loader.hpp"
#include "auto_apms_core/resources.hpp"
#include "rcpputils/split.hpp"

int main(int argc, char** argv)
{
    if (argc < 5) {
        std::cerr << "create_node_plugin_manifest: Missing inputs! The program requires: \n\t1.) the yaml "
                     "node manifest files (separated by ';').\n\t2.) Build information for nodes supposed to be "
                     "registered during build time (List of '<class_name>@<library_build_path>' "
                     "separated by ';').\n\t3.) The name of the package that provides the build targets.\n\t4.) Output "
                     "file for the complete node plugin manifest.\n\t";
        std::cerr << "Usage: create_node_plugin_manifest <manifest_files> <build_infos> <build_package_name> "
                     "<output_file>\n";
        return EXIT_FAILURE;
    }

    using PluginLoader = auto_apms_behavior_tree::BTNodePluginLoader;

    try {
        std::vector<std::string> manifest_files;
        for (const auto& path : rcpputils::split(argv[1], ';')) {
            manifest_files.push_back(std::filesystem::absolute(path).string());
        }
        const std::vector<std::string> build_infos = rcpputils::split(argv[2], ';');
        const std::string build_package_name = argv[3];
        const std::filesystem::path output_file{std::filesystem::absolute(argv[4])};

        // Ensure that arguments are not empty
        if (manifest_files.empty()) { throw std::runtime_error("Argument manifest_files must not be empty"); }
        if (output_file.empty()) { throw std::runtime_error("Argument output_file must not be empty."); }

        // Ensure correct extensions
        if (output_file.extension() != ".yaml") {
            throw std::runtime_error("Output file '" + output_file.string() +
                                     "' has wrong extension. Must be '.yaml'.");
        }

        rclcpp::init(argc, argv);
        auto node_ptr = std::make_shared<rclcpp::Node>("_create_node_plugin_manifest_temp_node");

#ifdef _AUTO_APMS_BEHAVIOR_TREE_DEBUG_LOGGING
        // Set logging severity
        auto ret = rcutils_logging_set_logger_level(node_ptr->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_ERROR(node_ptr->get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
            rcutils_reset_error();
        }
#endif

        // Retrieve plugin library paths from build info
        std::map<std::string, std::string> build_lib_paths;
        for (const auto& build_info : build_infos) {
            std::vector<std::string> parts = rcpputils::split(build_info, '@');
            if (parts.size() != 2) { throw std::runtime_error("Invalid build info entry ('" + build_info + "')."); }
            const std::string& class_name = parts[0];
            const std::string& build_path = parts[1];
            if (build_lib_paths.find(class_name) != build_lib_paths.end()) {
                throw std::runtime_error("Node class '" + class_name + "' is specified multiple times in build infos.");
            }
            build_lib_paths[class_name] = build_path;  // {class_name: build_path}
        }

        auto output_manifest = PluginLoader::Manifest::FromFiles(manifest_files);
        auto all_but_build_package =
            auto_apms_core::GetAllPackagesWithResource(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__NODE);
        all_but_build_package.erase(build_package_name);
        auto loader = PluginLoader{node_ptr, all_but_build_package};
        for (const auto& [node_name, params] : output_manifest.map()) {
            auto temp_manifest = PluginLoader::Manifest({{node_name, params}});
            try {
                loader.AutoCompleteManifest(temp_manifest);
                output_manifest[node_name] = temp_manifest[node_name];
            } catch (const auto_apms_behavior_tree::exceptions::ResourceNotFoundError& e) {
                if (build_lib_paths.find(params.class_name) == build_lib_paths.end()) {
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
        output_manifest.ToFile(output_file);

        // Print unique list of libraries to stdout
        std::set<std::string> paths;
        for (const auto& [node_name, params] : output_manifest.map()) {
            const auto& path = params.library;
            if (const auto& [_, success] = paths.insert(path); success) { std::cout << path << ';'; }
        }

    } catch (const std::exception& e) {
        std::cerr << "ERROR (create_node_plugin_manifest): " << e.what() << "\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
