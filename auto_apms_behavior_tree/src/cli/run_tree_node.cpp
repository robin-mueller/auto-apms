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

#include "auto_apms_behavior_tree/builder/tree_builder.hpp"
#include "auto_apms_behavior_tree/executor/executor.hpp"
#include "auto_apms_util/logging.hpp"
#include "auto_apms_util/yaml.hpp"
#include "rclcpp/rclcpp.hpp"

sig_atomic_t volatile termination_requested = 0;

using namespace std::chrono_literals;
using namespace auto_apms_behavior_tree;

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "run_tree_node: Missing inputs! The program requires: \n\t1.) YAML representation of "
                 "NodeRegistrationParams encoded in a string.\n\t2.) Optional: YAML map of specific node port values "
                 "encoded in a string.\n";
    std::cerr << "Usage: run_tree_node <registration_params> [<port_values>]\n";
    return EXIT_FAILURE;
  }

  // Ensure that rclcpp is not shut down before the tree has been halted (on destruction) and all pending actions have
  // been successfully canceled
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::SigTerm);
  signal(SIGINT, [](int /*sig*/) { termination_requested = 1; });
  auto node_ptr = std::make_shared<rclcpp::Node>("run_tree_node_cpp");
  auto_apms_util::exposeToDebugLogging(node_ptr->get_logger());

  core::NodeRegistrationParams registration_params;
  try {
    registration_params = core::NodeRegistrationParams::decode(argv[1]);
  } catch (std::exception & e) {
    RCLCPP_ERROR(node_ptr->get_logger(), "ERROR interpreting argument registration_params: %s", e.what());
    return EXIT_FAILURE;
  }

  TreeBuilder::PortValues port_values;
  if (argc > 2) {
    try {
      port_values = auto_apms_util::yamlToMap(argv[2]);
    } catch (std::exception & e) {
      RCLCPP_ERROR(node_ptr->get_logger(), "ERROR interpreting argument port_values: %s", e.what());
      return EXIT_FAILURE;
    }
  }

  const std::string tree_name = "RunTreeNodeCPP";
  TreeBuilder builder(node_ptr);
  try {
    auto tree_element = builder.insertNewTreeElement(tree_name);
    auto node_element =
      builder.insertNewNodeElement(tree_element, registration_params.class_name.c_str(), registration_params);
    builder.addNodePortValues(node_element, port_values, true);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_ptr->get_logger(), "ERROR inserting tree node: %s", e.what());
    return EXIT_FAILURE;
  }

  RCLCPP_INFO(
    node_ptr->get_logger(), "Creating a tree with a single node:\n%s", builder.writeTreeDocumentToString().c_str());

  TreeExecutor executor(node_ptr);
  auto future = executor.startExecution(
    [&builder, &tree_name](TreeBlackboardSharedPtr bb) { return builder.instantiateTree(tree_name, bb); });

  const auto termination_timeout = std::chrono::duration<double>(1.5);
  rclcpp::Time termination_start;
  bool termination_started = false;
  try {
    while (rclcpp::spin_until_future_complete(node_ptr, future, std::chrono::milliseconds(250)) !=
           rclcpp::FutureReturnCode::SUCCESS) {
      if (termination_started) {
        if (node_ptr->now() - termination_start > termination_timeout) {
          RCLCPP_WARN(node_ptr->get_logger(), "Termination took too long. Aborted.");
          return EXIT_FAILURE;
        }
      } else if (termination_requested) {
        termination_start = node_ptr->now();
        executor.setControlCommand(TreeExecutor::ControlCommand::TERMINATE);
        termination_started = true;
        RCLCPP_INFO(node_ptr->get_logger(), "Terminating tree execution...");
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_ptr->get_logger(), "ERROR during behavior tree execution: %s", e.what());
    return EXIT_FAILURE;
  }

  // To prevent a deadlock, throw if future isn't ready at this point. However, this shouldn't happen.
  if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    throw std::logic_error("Future object is not ready.");
  }

  RCLCPP_INFO(node_ptr->get_logger(), "Finished with status %s.", toStr(future.get()).c_str());
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
