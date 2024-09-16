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

#include <iostream>

#include "auto_apms/resource/node.hpp"
#include "rcpputils/split.hpp"

int main(int argc, char** argv)
{
    using namespace auto_apms;

    if (argc < 4) {
        std::cerr << "get_nodes_require_build_info: Missing inputs! The program requires: \n\t1.) the yaml "
                     "configuration files (separated by ';')\n\t2.) Build information for nodes supposed to be "
                     "registered during compile time (List of '<class_name>@<library_build_path>' "
                     "separated by ';')\n\t3.) The name of the package that provides the build info\n";
        std::cerr << "Usage: get_nodes_require_build_info <registration_config_files> <build_infos> "
                     "<build_package_name>\n";
        return EXIT_FAILURE;
    }
    std::vector<std::string> registration_config_files = rcpputils::split(argv[1], ';');
    std::vector<std::string> build_infos = rcpputils::split(argv[2], ';');
    std::string build_package_name = argv[3];

    // Ensure that arguments are not empty
    if (registration_config_files.empty()) {
        throw std::runtime_error("get_nodes_require_build_info: Argument registration_config_files must not be empty");
    }

    // Retrieve plugin library paths from build info of the current package
    std::map<std::string, std::string> build_lib_paths;
    for (const auto& build_info : build_infos) {
        std::vector<std::string> parts = rcpputils::split(build_info, '@');
        if (parts.size() != 2) {
            throw std::runtime_error("get_nodes_require_build_info: Invalid build info entry: " + build_info);
        }
        build_lib_paths[parts[0]] = parts[1];  // {class_name: library_build_path}
    }

    // Determine which nodes are being compiled by package build_package_name thus require additional paths when
    // registered during build time
    for (const auto& node : resource::ParseBTNodeRegistrationConfig(registration_config_files)) {
        const auto& name = node.first;
        const auto& config = node.second;
        if (build_lib_paths.find(config.class_name) != build_lib_paths.end() && !config.library.has_value() &&
            config.package.value_or(build_package_name) == build_package_name) {
            std::cout << name << '@' << build_lib_paths[config.class_name] << ';';
        }
    }

    return EXIT_SUCCESS;
}
