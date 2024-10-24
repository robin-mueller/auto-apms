// Copyright 2024 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <signal.h>

#include <chrono>

#include "auto_apms_behavior_tree/creator.hpp"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "rclcpp/rclcpp.hpp"

sig_atomic_t volatile shutdown_requested = 0;

using namespace std::chrono_literals;
using namespace auto_apms_behavior_tree;

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "run_behavior_tree: Missing inputs! The program requires: \n\t1.) Name of the registered behavior "
                     "tree file (extension may be omitted).\n\t2.) Optional: Name of the package to be searched "
                     "(Default is \"\": Searching in all packages).\n\t3.) Optional: ID of the tree to be executed "
                     "(Default is \"\": Using the main tree).\n";
        std::cerr << "Usage: run_behavior_tree <file_name> [<package_name>] [<tree_id>]\n";
        return EXIT_FAILURE;
    }
    const std::string tree_file_name{argv[1]};
    const std::string package_name{argc > 2 ? argv[2] : ""};
    const std::string tree_id{argc > 3 ? argv[3] : ""};

    // Ensure that rclcpp is not shut down before the tree has been halted (on destruction) and all pending actions have
    // been successfully canceled
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::SigTerm);
    signal(SIGINT, [](int sig) {
        (void)sig;
        shutdown_requested = 1;
    });
    auto node = std::make_shared<rclcpp::Node>("run_behavior_tree");

    std::unique_ptr<BTResource> tree_resource_ptr;
    try {
        tree_resource_ptr = std::make_unique<BTResource>(BTResource::SelectByFileName(tree_file_name, package_name));
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(),
                     "ERROR searching for corresponding behavior tree resource with arguments tree_file_name: '%s', "
                     "package_name: '%s': %s",
                     tree_file_name.c_str(),
                     package_name.c_str(),
                     e.what());
        return EXIT_FAILURE;
    }

    BT::Tree tree;
    std::string main_tree_id;
    try {
        auto tree_creator_ptr = BTCreator::FromResource(node, *tree_resource_ptr);
        tree_creator_ptr->SetMainTreeName(tree_id);
        main_tree_id = tree_creator_ptr->GetMainTreeName();
        tree = tree_creator_ptr->CreateMainTree();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(),
                     "ERROR creating behavior tree '%s' from path %s: %s",
                     tree_id.c_str(),
                     tree_resource_ptr->tree_file_path.c_str(),
                     e.what());
        return EXIT_FAILURE;
    }

    // Prepare execution
    BT::Groot2Publisher publisher(tree);
    BT::StdCoutLogger logger(tree);
    logger.enableTransitionToIdle(false);
    auto status = BT::NodeStatus::IDLE;

    RCLCPP_INFO(node->get_logger(),
                "Running tree '%s::%s' ...",
                tree_resource_ptr->package_name.c_str(),
                main_tree_id.c_str());

    try {
        while (!shutdown_requested && status <= BT::NodeStatus::RUNNING) {
            status = tree.tickOnce();
            tree.sleep(100ms);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "ERROR during behavior tree tick: %s", e.what());
        return EXIT_FAILURE;
    }

    RCLCPP_INFO(node->get_logger(), "Finished with status %s", BT::toStr(status).c_str());

    rclcpp::shutdown();

    return 0;
}
