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

#include "auto_apms_behavior_tree/executor/executor_base.hpp"
#include "auto_apms_behavior_tree_core/builder.hpp"
#include "auto_apms_util/logging.hpp"
#include "auto_apms_util/string.hpp"
#include "rclcpp/rclcpp.hpp"

sig_atomic_t volatile termination_requested = 0;

using namespace auto_apms_behavior_tree;

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr
      << "run_tree: Missing inputs! The program requires: \n\t1.) The identity string of an installed tree resource.\n";
    std::cerr << "Usage: run_tree <tree_identity>\n";
    return EXIT_FAILURE;
  }
  const core::TreeResourceIdentity tree_identity(auto_apms_util::trimWhitespaces(argv[1]));

  // Ensure that rclcpp is not shut down before the tree has been halted (on destruction) and all pending actions have
  // been successfully canceled
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::SigTerm);
  signal(SIGINT, [](int /*sig*/) { termination_requested = 1; });
  rclcpp::Node::SharedPtr node_ptr = std::make_shared<rclcpp::Node>("run_tree_cpp");
  auto_apms_util::exposeToDebugLogging(node_ptr->get_logger());

  std::unique_ptr<core::TreeResource> tree_resource_ptr;
  try {
    tree_resource_ptr = std::make_unique<core::TreeResource>(tree_identity);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_ptr->get_logger(), "ERROR searching for behavior tree resource '%s': %s", tree_identity.str().c_str(),
      e.what());
    return EXIT_FAILURE;
  }

  TreeExecutorBase executor(node_ptr, nullptr, true);
  core::TreeBuilder builder(
    node_ptr, executor.getTreeNodeWaitablesCallbackGroupPtr(), executor.getTreeNodeWaitablesExecutorPtr());
  try {
    builder.newTreeFromResource(*tree_resource_ptr).makeRoot();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_ptr->get_logger(), "ERROR loading behavior tree from resource '%s': %s", tree_identity.str().c_str(),
      e.what());
    return EXIT_FAILURE;
  }

  std::shared_future<TreeExecutorBase::ExecutionResult> future =
    executor.startExecution([&builder](TreeBlackboardSharedPtr bb) { return builder.instantiate(bb); });

  RCLCPP_INFO(
    node_ptr->get_logger(), "Executing tree with identity '%s::%s::%s'.", tree_resource_ptr->getPackageName().c_str(),
    tree_resource_ptr->getFileStem().c_str(), builder.getRootTree().getName().c_str());

  const std::chrono::duration<double> termination_timeout(2);
  rclcpp::Time termination_start;
  bool termination_started = false;
  while (rclcpp::spin_until_future_complete(node_ptr, future, std::chrono::milliseconds(250)) !=
         rclcpp::FutureReturnCode::SUCCESS) {
    if (termination_started) {
      if (node_ptr->now() - termination_start > termination_timeout) {
        RCLCPP_WARN(node_ptr->get_logger(), "Termination took too long. Aborted.");
        return EXIT_FAILURE;
      }
    } else if (termination_requested) {
      termination_start = node_ptr->now();
      executor.setControlCommand(TreeExecutorBase::ControlCommand::TERMINATE);
      termination_started = true;
      RCLCPP_INFO(node_ptr->get_logger(), "Terminating tree execution...");
    }
  }

  // To prevent a deadlock, throw if future isn't ready at this point. However, this shouldn't happen.
  if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    throw std::logic_error("Future object is not ready.");
  }

  RCLCPP_INFO(node_ptr->get_logger(), "Finished with status %s.", toStr(future.get()).c_str());
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
