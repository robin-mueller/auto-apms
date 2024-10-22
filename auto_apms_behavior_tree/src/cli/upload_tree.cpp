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

#include "auto_apms_behavior_tree/bt_executor_client.hpp"
#include "auto_apms_core/util/console.hpp"

using namespace auto_apms_core::util;
using namespace auto_apms_behavior_tree;

int main(int argc, char* argv[])
{
    if (argc < 5) {
        std::cerr << "upload_tree: Missing inputs! The program requires: \n\t1.) the namespace of the executor node\n\t"
                     "2.) the name of the executor to register behavior trees with\n\t3.) the name of the package that "
                     "provides the behavior trees file\n\t4.) the name of the file containing the xml data\n\t5.) "
                     "Optional: the ID of the tree to execute\n";
        std::cerr
            << "Usage: register_tree <namespace> <executor_name> <package_name> <tree_file_name> [<main_tree_id>]\n";
        return EXIT_FAILURE;
    }
    const std::string namespace_{argv[1]};
    const std::string executor_name{argv[2]};
    const std::string package_name{argv[3]};
    const std::string tree_file_name{argv[4]};
    const std::string main_tree_id{argc > 5 ? argv[5] : ""};

    std::cout << "Uploading behavior tree to executor '" << ColoredText(executor_name, TextColor::CYAN)
              << "' in namespace '" << ColoredText(namespace_, TextColor::CYAN) << "'"
              << "\n\tpackage_name  \t'" << ColoredText(package_name, TextColor::CYAN) << "'"
              << "\n\ttrees_filename\t'" << ColoredText(tree_file_name, TextColor::CYAN) << "'"
              << "\n\ttree_id       \t'"
              << ColoredText(main_tree_id.empty() ? "[main_tree_to_execute]" : main_tree_id, TextColor::CYAN) << "'"
              << std::endl;

    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<rclcpp::Node>(executor_name + "_upload_node", namespace_);

    // Restrict ROS2 logging output
    auto ret = rcutils_logging_set_logger_level(node_ptr->get_logger().get_name(), RCUTILS_LOG_SEVERITY_WARN);
    if (ret != RCUTILS_RET_OK) {
        std::cerr << "Error setting ROS2 logging severity: " << rcutils_get_error_string().str << '\n';
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    // Create behavior tree executor client
    auto bt_executor_client = BTExecutorClient(*node_ptr, executor_name);

    // Register all behavior trees that are defined in the file with the executor
    if (bt_executor_client.UploadBehaviorTree(BTResource::SelectByFileName(tree_file_name, package_name),
                                              main_tree_id)) {
        std::cout << " --> " << ColoredText("Registration successful", TextColor::GREEN) << std::endl;
    }
    else {
        std::cout << " --> " << ColoredText("Registration failed", TextColor::RED) << std::endl;
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
