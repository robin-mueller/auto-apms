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
            << "generate_bt_node_model: Missing inputs! The program requires: \n\t1.) the yaml configuration files "
               "(separated by ';')\n\t2.) Output file for the registration manifest\n\t3.) Optional: The name of the "
               "package that builds the behavior tree model \n\t4.) Optional: "
               "Additional paths for plugin libraries being built by the same package that issues the model generation "
               "(separated by ';')\n";
        std::cerr << "Usage: create_bt_node_registration_manifest <registration_config_files> <output_file> "
                     "[<build_package_name> <build_infos>]\n";
        return EXIT_FAILURE;
    }
    std::vector<std::string> registration_config_files = rcpputils::split(argv[1], ';');
    std::filesystem::path output_file{std::filesystem::absolute(argv[2])};
    std::string build_package_name;
    std::vector<std::string> build_infos;
    if (argc > 4) {
        // Both <build_package_name> and <build_infos> must be specified to consider the build paths for the manifest
        build_package_name = argv[3];
        build_infos = rcpputils::split(argv[4], ';');
    }

    // Ensure that arguments are not empty
    if (registration_config_files.empty()) {
        throw std::runtime_error("Argument registration_config_files must not be empty");
    }
    if (output_file.empty()) { throw std::runtime_error("Argument output_file must not be empty"); }

    // Ensure correct extensions
    if (output_file.extension().compare(".yaml") != 0) {
        throw std::runtime_error("Output file '" + output_file.string() + "' has wrong extension. Must be '.yaml'");
    }

    // Retrieve plugin library paths from build info of the current package
    std::map<std::string, std::string> build_lib_paths;
    for (const auto& build_info : build_infos) {
        std::vector<std::string> parts = rcpputils::split(build_info, '@');
        if (parts.size() != 2) { throw std::runtime_error("Invalid build info entry: " + build_info); }
        const std::string& class_name = parts[0];
        const std::string& path = parts[1];
        build_lib_paths[class_name] = path;
    }

    // Fill library parameter with preferred build info if applicable
    resource::BTNodeRegistrationConfigMap registration_config_map =
        resource::ParseBTNodeRegistrationConfig(registration_config_files);
    for (auto& it : registration_config_map) {
        auto& reg_config = it.second;
        if (build_lib_paths.find(reg_config.class_name) != build_lib_paths.end() && !reg_config.library.has_value() &&
            reg_config.package.value_or(build_package_name) == build_package_name) {
            reg_config.library = build_lib_paths[reg_config.class_name];
            reg_config.package = std::nullopt;
        }
    }

    const auto manifest = resource::CreateBTNodeRegistrationManifest(registration_config_map);
    registration_config_map.clear();

    // Create new registration config
    std::set<std::string> libs;
    for (const auto& it : manifest) {
        const auto& m = it.second;
        resource::BTNodeRegistrationConfig config;
        config = m.registration_config;
        config.library = m.library_path;
        config.package = m.package_name.empty() ? std::nullopt : std::optional<std::string>{m.package_name};
        registration_config_map[it.first] = config;
        if (const auto ret = libs.insert(m.library_path); ret.second) std::cout << m.library_path << ';';
    }

    YAML::Node root;
    root = registration_config_map;

    // Write the manifest file
    std::ofstream out_stream{output_file};
    if (out_stream.is_open()) {
        out_stream << root;
        out_stream.close();
    }
    else {
        std::cerr << "create_bt_node_registration_manifest: Error opening registration manifest output file '"
                  << output_file << "'\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
