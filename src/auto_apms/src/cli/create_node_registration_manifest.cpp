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
#include <fstream>
#include <iostream>

#include "auto_apms/resource/node.hpp"
#include "rcpputils/split.hpp"
#include "yaml-cpp/yaml.h"

int main(int argc, char** argv)
{
    using namespace auto_apms;

    if (argc < 3) {
        std::cerr
            << "create_node_registration_manifest: Missing inputs! The program requires: \n\t1.) the yaml "
               "configuration files "
               "(separated by ';')\n\t2.) Output file for the registration manifest\n\t3.) Optional: Build information "
               "for nodes supposed to be registered during compile time (List of '<node_name>@<library_build_path>' "
               "separated by ';')\n";
        std::cerr << "Usage: create_node_registration_manifest <registration_config_files> <output_file> "
                     "[<build_infos>]\n";
        return EXIT_FAILURE;
    }
    std::vector<std::string> registration_config_files = rcpputils::split(argv[1], ';');
    std::filesystem::path output_file{std::filesystem::absolute(argv[2])};
    std::vector<std::string> build_infos;
    if (argc > 3) { build_infos = rcpputils::split(argv[3], ';'); }

    // Ensure that arguments are not empty
    if (registration_config_files.empty()) {
        throw std::runtime_error(
            "create_node_registration_manifest: Argument registration_config_files must not be empty");
    }
    if (output_file.empty()) {
        throw std::runtime_error("create_node_registration_manifest: Argument output_file must not be empty");
    }

    // Ensure correct extensions
    if (output_file.extension().compare(".yaml") != 0) {
        throw std::runtime_error("create_node_registration_manifest: Output file '" + output_file.string() +
                                 "' has wrong extension. Must be '.yaml'");
    }

    // Retrieve plugin library paths from build info
    std::map<std::string, std::string> build_lib_paths;
    for (const auto& build_info : build_infos) {
        std::vector<std::string> parts = rcpputils::split(build_info, '@');
        if (parts.size() != 2) {
            throw std::runtime_error("create_node_registration_manifest: Invalid build info entry: " + build_info);
        }
        build_lib_paths[parts[0]] = parts[1];  // {node_name: library_build_path}
    }

    // Fill library parameter with preferred build info if applicable
    auto manifest_map = resource::ParseBTNodeManifestFile(registration_config_files);
    for (auto& build_info : build_lib_paths) {
        if (manifest_map.find(build_info.first) == manifest_map.end()) {
            std::cerr << "create_node_registration_manifest: Build info contains node '" + build_info.first +
                             "' which is not specified in the registration map";
            return EXIT_FAILURE;
        }
        manifest_map[build_info.first].library = build_info.second;
        manifest_map[build_info.first].package = std::nullopt;  // Indicate that library is inferred from build info
    }

    const auto validated_manifest_map = resource::ValidateBTNodeManifest(manifest_map);

    YAML::Node root;
    root = validated_manifest_map;

    // Write the manifest file
    std::ofstream out_stream{output_file};
    if (out_stream.is_open()) {
        out_stream << root;
        out_stream.close();
    }
    else {
        std::cerr << "create_node_registration_manifest: Error opening registration manifest output file '"
                  << output_file << "'\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
