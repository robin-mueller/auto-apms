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

#include "auto_apms_behavior_tree/executor/executor.hpp"
#include "auto_apms_behavior_tree/builder/tree_builder.hpp"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "rclcpp/rclcpp.hpp"

sig_atomic_t volatile termination_requested = 0;

using namespace std::chrono_literals;
using namespace auto_apms_behavior_tree;

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "run_behavior_tree: Missing inputs! The program requires: \n\t1.) Name of the registered behavior "
                 "tree file (extension may be omitted).\n\t2.) Optional: Name of the tree to be executed "
                 "(Default is \"\": Using the main tree).\n\t3.) Optional: Name of the package to be searched "
                 "(Default is \"\": Searching in all packages).\n";
    std::cerr << "Usage: run_behavior_tree <file_name> [<tree_name>] [<package_name>]\n";
    return EXIT_FAILURE;
  }
  const std::string tree_file_name{ argv[1] };
  const std::string tree_name{ argc > 2 ? argv[2] : "" };
  const std::string package_name{ argc > 3 ? argv[3] : "" };

  // Ensure that rclcpp is not shut down before the tree has been halted (on destruction) and all pending actions have
  // been successfully canceled
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::SigTerm);
  signal(SIGINT, [](int sig) {
    (void)sig;
    termination_requested = 1;
  });
  auto node = std::make_shared<rclcpp::Node>("run_behavior_tree");

#ifdef _AUTO_APMS_DEBUG_LOGGING
  auto ret = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK)
  {
    std::cerr << "Error setting ROS2 logging severity: " << rcutils_get_error_string().str << '\n';
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
#endif

  std::unique_ptr<TreeResource> tree_resource_ptr;
  try
  {
    tree_resource_ptr = std::make_unique<TreeResource>(TreeResource::SelectByFileName(tree_file_name, package_name));
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "ERROR searching for corresponding behavior tree resource with arguments tree_file_name: '%s', "
                 "package_name: '%s': %s",
                 tree_file_name.c_str(), package_name.c_str(), e.what());
    return EXIT_FAILURE;
  }

  TreeBuilder builder;
  try
  {
    builder.addTreeFromResource(*tree_resource_ptr, node);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node->get_logger(), "ERROR loading behavior tree '%s' from resource %s: %s", tree_name.c_str(),
                 tree_resource_ptr->tree_file_path.c_str(), e.what());
    return EXIT_FAILURE;
  }

  BTExecutorBase executor{ node };
  auto future =
      executor.Start([&builder, &tree_name](BT::Blackboard::Ptr bb) { return builder.getTree(tree_name, bb); });

  RCLCPP_INFO(node->get_logger(), "Executing tree with identity '%s::%s::%s'.",
              tree_resource_ptr->tree_file_stem.c_str(), builder.GetMainTreeName().c_str(),
              tree_resource_ptr->package_name.c_str());

  const auto termination_timeout = std::chrono::duration<int, std::milli>(1500);
  std::chrono::steady_clock::time_point termination_start;
  bool termination_started = false;
  try
  {
    while (rclcpp::spin_until_future_complete(node, future, std::chrono::milliseconds(250)) !=
           rclcpp::FutureReturnCode::SUCCESS)
    {
      if (termination_started)
      {
        if (std::chrono::steady_clock::now() - termination_start > termination_timeout)
        {
          RCLCPP_WARN(node->get_logger(), "Termination took too long. Aborted.");
          return EXIT_FAILURE;
        }
      }
      else if (termination_requested)
      {
        termination_start = std::chrono::steady_clock::now();
        executor.set_control_command(BTExecutorBase::ControlCommand::TERMINATE);
        termination_started = true;
        RCLCPP_INFO(node->get_logger(), "Terminating tree execution...");
      }
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node->get_logger(), "ERROR during behavior tree execution: %s", e.what());
    return EXIT_FAILURE;
  }

  // To prevent a deadlock, throw if future isn't ready at this point. However, this shouldn't happen.
  if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    throw std::logic_error("Future object is not ready.");
  }

  RCLCPP_INFO(node->get_logger(), "Finished with status %s.", to_string(future.get()).c_str());
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
