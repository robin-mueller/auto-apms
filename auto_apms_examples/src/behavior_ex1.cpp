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
#include <filesystem>

#include "auto_apms_behavior_tree/behavior_tree.hpp"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "rclcpp/rclcpp.hpp"

sig_atomic_t volatile shutdown_requested = 0;

using namespace std::chrono_literals;
using namespace auto_apms_behavior_tree;

int main(int argc, char** argv)
{
    // Ensure that rclcpp is not shut down before the tree has been halted (on destruction) and all pending actions have
    // been successfully canceled
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::SigTerm);
    signal(SIGINT, [](int sig) {
        (void)sig;
        shutdown_requested = 1;
    });
    auto node = std::make_shared<rclcpp::Node>(std::string(EXAMPLE_NAME) + "_node");

    {
        BT::Tree tree =
            BehaviorTree::Create(node,
                                 BehaviorTreeResource::SelectByFileName("relative_goto", "auto_apms_examples"),
                                 "");
        BT::Groot2Publisher publisher(tree);
        BT::StdCoutLogger logger(tree);
        logger.enableTransitionToIdle(false);

        auto status = BT::NodeStatus::IDLE;
        while (!shutdown_requested && status <= BT::NodeStatus::RUNNING) {
            status = tree.tickOnce();
            tree.sleep(1ms);
        }

        std::cout << "Finished tree with status: " << BT::toStr(status) << std::endl;
    }

    rclcpp::shutdown();

    return 0;
}
