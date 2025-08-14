// Copyright 2025 Robin Müller
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

/**
 * @example simple_skill_node.cpp
 *
 * Example for implementing a skill that simply prints a message a specific amount of times to the terminal. Both a
 * server and a client are created using AutoAPMS.
 *
 * @sa <a
 * href="https://robin-mueller.github.io/auto-apms-guide/tutorial/creating-a-behavior-from-scratch">
 * Tutorial: Creating a Behavior From Scratch</a>
 */

#include <chrono>

#include "auto_apms_interfaces/action/example_simple_skill.hpp"
#include "auto_apms_util/action_wrapper.hpp"

/**
 * Implement the ROS 2 node acting as the server for the skill
 */

namespace auto_apms_examples
{

using SimpleSkillActionType = auto_apms_interfaces::action::ExampleSimpleSkill;

class SimpleSkillServer : public auto_apms_util::ActionWrapper<SimpleSkillActionType>
{
public:
  SimpleSkillServer(const rclcpp::NodeOptions & options) : ActionWrapper("simple_skill", options) {}

  // Callback invoked when a goal arrives
  bool onGoalRequest(std::shared_ptr<const Goal> /*goal_ptr*/) override final
  {
    index_ = 1;
    start_ = node_ptr_->now();
    return true;
  }

  // Callback invoked asynchronously by the internal execution routine
  Status executeGoal(
    std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Feedback> /*feedback_ptr*/,
    std::shared_ptr<Result> result_ptr) override final
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "#%i - %s", index_++, goal_ptr->msg.c_str());
    if (index_ <= goal_ptr->n_times) {
      return Status::RUNNING;
    }
    result_ptr->time_required = (node_ptr_->now() - start_).to_chrono<std::chrono::duration<double>>().count();
    return Status::SUCCESS;
  }

private:
  uint8_t index_;
  rclcpp::Time start_;
};

}  // namespace auto_apms_examples

// Register the skill as a ROS 2 component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_examples::SimpleSkillServer)

/**
 * Implement the behavior tree node acting as the client for the skill
 */

#include "auto_apms_behavior_tree/node.hpp"

namespace auto_apms_examples
{

class SimpleSkillClient : public auto_apms_behavior_tree::core::RosActionNode<SimpleSkillActionType>
{
public:
  using RosActionNode::RosActionNode;

  // We must define data ports to accept arguments
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("msg"), BT::InputPort<uint8_t>("n_times")};
  }

  // Callback invoked to specify the action goal
  bool setGoal(Goal & goal) override final
  {
    RCLCPP_INFO(logger_, "--- Set goal ---");
    goal.msg = getInput<std::string>("msg").value();
    goal.n_times = getInput<uint8_t>("n_times").value();
    return true;
  }

  // Callback invoked when the action is finished
  BT::NodeStatus onResultReceived(const WrappedResult & result) override final
  {
    RCLCPP_INFO(logger_, "--- Result received ---");
    RCLCPP_INFO(logger_, "Time elapsed: %f", result.result->time_required);
    return RosActionNode::onResultReceived(result);
  }
};

}  // namespace auto_apms_examples

// Make the node discoverable for the class loader
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_examples::SimpleSkillClient)
