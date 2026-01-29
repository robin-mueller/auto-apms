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

#include "auto_apms_behavior_tree/executor/executor_node.hpp"
#include "auto_apms_util/logging.hpp"
#include "auto_apms_util/string.hpp"
#include "rclcpp/rclcpp.hpp"

sig_atomic_t volatile termination_requested = 0;

using namespace auto_apms_behavior_tree;

int main(int argc, char ** argv)
{
  const std::vector<std::string> args_vector = rclcpp::remove_ros_arguments(argc, argv);

  bool print_help = false;
  std::string build_request = "";
  std::string entry_point = "";
  core::NodeManifest node_manifest;
  if (args_vector.size() > 1) {
    const std::string & arg = args_vector[1];
    print_help = "-h" == arg || "--help" == arg;
    if (!print_help) build_request = auto_apms_util::trimWhitespaces(arg);
  }
  if (args_vector.size() > 2) {
    const std::string & arg = args_vector[2];
    print_help = "-h" == arg || "--help" == arg;
    if (!print_help) entry_point = auto_apms_util::trimWhitespaces(arg);
  }
  if (args_vector.size() > 3) {
    const std::string & arg = args_vector[3];
    print_help = "-h" == arg || "--help" == arg;
    if (!print_help) node_manifest = core::NodeManifest::decode(auto_apms_util::trimWhitespaces(arg));
  }
  if (print_help) {
    std::cerr << "run_behavior: The program accepts: \n\t1.) String specifying the behavior tree "
                 "build request to be passed to the build handler loaded by the underlying tree executor node. If "
                 "empty, build handler must be able to handle that.\n\t2.) Optional string specifying the single point "
                 "of entry for behavior execution.\n\t3.) Optional encoded node manifest specifying the behavior tree "
                 "nodes required for behavior execution.\n";
    std::cerr << "Usage: run_behavior [<build_request>] [<entry_point>] [<node_manifest>]\n";
    return EXIT_FAILURE;
  }

  // Ensure that rclcpp is not shut down before the tree has been halted (on destruction) and all pending actions have
  // been successfully canceled
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::SigTerm);
  signal(SIGINT, [](int /*sig*/) { termination_requested = 1; });

  // Create executor node
  rclcpp::NodeOptions opt;
  TreeExecutorNodeOptions executor_opt(opt);
  TreeExecutorNode executor("run_behavior", executor_opt);
  const rclcpp::Logger logger = executor.getNodePtr()->get_logger();

  // Start tree execution
  std::shared_future<TreeExecutorBase::ExecutionResult> future =
    executor.startExecution(build_request, entry_point, node_manifest);
  RCLCPP_INFO(logger, "Executing tree '%s'.", executor.getTreeName().c_str());

  const std::chrono::duration<double> termination_timeout(3);
  rclcpp::Time termination_start;
  bool termination_started = false;
  while (
    rclcpp::spin_until_future_complete(executor.get_node_base_interface(), future, std::chrono::milliseconds(250)) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    if (termination_started) {
      if (executor.getNodePtr()->now() - termination_start > termination_timeout) {
        RCLCPP_WARN(logger, "Termination took too long. Aborted.");
        return EXIT_FAILURE;
      }
    } else if (termination_requested) {
      termination_start = executor.getNodePtr()->now();
      executor.setControlCommand(TreeExecutorBase::ControlCommand::TERMINATE);
      termination_started = true;
      RCLCPP_INFO(logger, "Terminating tree execution...");
    }
  }

  // To prevent a deadlock, throw if future isn't ready at this point. However, this shouldn't happen.
  if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    throw std::logic_error("Future object is not ready.");
  }

  RCLCPP_INFO(logger, "Finished with status %s.", toStr(future.get()).c_str());
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
