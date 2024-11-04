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

#include <gtest/gtest.h>

#include <chrono>

#include "auto_apms_interfaces/action/test_action_wrapper.hpp"
#include "auto_apms_util/action_client_wrapper.hpp"
#include "auto_apms_util/action_wrapper.hpp"

using namespace std::chrono_literals;
using namespace auto_apms_util;
using ActionType = auto_apms_interfaces::action::TestActionWrapper;

class TestAction : public ActionWrapper<ActionType>
{
public:
  using ActionWrapper::ActionWrapper;

  bool onGoalRequest(std::shared_ptr<const Goal> goal_ptr) override
  {
    if (goal_ptr->request == 1)
    {
      return false;
    }
    return true;
  }

  void setInitialResult(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr) override
  {
    if (goal_ptr->request == 2)
    {
      result_ptr->result = 1;
    }
    if (goal_ptr->request == 3)
    {
      result_ptr->result = 10;
    }
  }
  bool onCancelRequest(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr) override
  {
    if (goal_ptr->request == 5)
    {
      return false;
    }
    if (goal_ptr->request == 7)
    {
      result_ptr->result = 20;
    }
    return true;
  }
  ActionStatus cancelGoal(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr) override
  {
    switch (goal_ptr->request)
    {
      case 6:
        // CancelGoalReject
        if (++result_ptr->result < 23)
        {
          return ActionStatus::RUNNING;
        }
        else
        {
          return ActionStatus::SUCCESS;
        }
      case 8:
        // CancelGoalAbort
        result_ptr->result = 40;
        return ActionStatus::FAILURE;
      default:
        throw std::logic_error("Unexpected goal request in cancelGoal: " + std::to_string(goal_ptr->request));
    }
  }
  ActionStatus executeGoal(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Feedback> feedback_ptr,
                           std::shared_ptr<Result> result_ptr) override
  {
    switch (goal_ptr->request)
    {
      case 2:
        // InitialResult
        return ActionStatus::SUCCESS;
      case 3:
        // ExecuteGoal
        if (++result_ptr->result < 13)
        {
          return ActionStatus::RUNNING;
        }
        else
        {
          return ActionStatus::SUCCESS;
        }
      case 4:
        // SendFeedback
        feedback_ptr->feedback = 1;
        return feeback_received ? ActionStatus::SUCCESS : ActionStatus::RUNNING;
      case 5:
        // CancelGoalReject
        return ActionStatus::RUNNING;
      case 6:
        // CancelGoalAccept
        return ActionStatus::RUNNING;
      case 7:
        // ExecuteGoalAbort
        result_ptr->result = 30;
        return ActionStatus::FAILURE;
      case 8:
        // CancelGoalAbort
        return ActionStatus::RUNNING;
      default:
        throw std::logic_error("Unexpected goal request in executeGoal: " + std::to_string(goal_ptr->request));
    }
  }

  bool feeback_received = false;
};

class TestFixture : public testing::Test
{
protected:
  TestFixture()
  {
    rclcpp::NodeOptions opt;
    opt.append_parameter_override(ACTION_WRAPPER_PARAM_NAME_LOOP_RATE, 0.001);
    opt.append_parameter_override(ACTION_WRAPPER_PARAM_NAME_FEEDBACK_RATE, 0.001);
    node_ptr = std::make_shared<rclcpp::Node>("test_action_wrapper_node", opt);
    action_server_ptr = std::make_shared<TestAction>("test_action_wrapper", node_ptr);
    action_client_ptr = std::make_shared<ActionClientWrapper<ActionType>>(node_ptr, "test_action_wrapper");
  }

  rclcpp::Node::SharedPtr node_ptr;
  std::shared_ptr<TestAction> action_server_ptr;
  TestAction::Goal goal;
  std::shared_ptr<ActionClientWrapper<ActionType>> action_client_ptr;
};

TEST_F(TestFixture, RejectGoal)
{
  goal.request = 1;

  auto response_future = action_client_ptr->syncSendGoal(goal);
  ASSERT_EQ(action_client_ptr->getGoalStatus(response_future), ActionGoalStatus::REJECTED) << "Goal was not rejected";
}

TEST_F(TestFixture, InitialResult)
{
  goal.request = 2;

  auto response_future = action_client_ptr->syncSendGoal(goal);
  ASSERT_NE(action_client_ptr->getGoalStatus(response_future), ActionGoalStatus::REJECTED) << "Goal was rejected";
  ASSERT_EQ(rclcpp::spin_until_future_complete(node_ptr, response_future), rclcpp::FutureReturnCode::SUCCESS) << "spin_"
                                                                                                                 "until"
                                                                                                                 "_"
                                                                                                                 "futur"
                                                                                                                 "e_"
                                                                                                                 "compl"
                                                                                                                 "ete "
                                                                                                                 "did "
                                                                                                                 "not "
                                                                                                                 "succe"
                                                                                                                 "ed";
  auto result = response_future.get();
  ASSERT_TRUE(result) << "result is nullptr after completion";
  EXPECT_EQ(result->code, rclcpp_action::ResultCode::SUCCEEDED) << "ActionWrapper did not succeed";
  ASSERT_EQ(result->result->result, 1) << "Result has unexpected value";
}

TEST_F(TestFixture, ExecuteGoal)
{
  goal.request = 3;

  auto response_future = action_client_ptr->syncSendGoal(goal);
  ASSERT_NE(action_client_ptr->getGoalStatus(response_future), ActionGoalStatus::REJECTED) << "Goal was rejected";
  ASSERT_EQ(rclcpp::spin_until_future_complete(node_ptr, response_future), rclcpp::FutureReturnCode::SUCCESS) << "spin_"
                                                                                                                 "until"
                                                                                                                 "_"
                                                                                                                 "futur"
                                                                                                                 "e_"
                                                                                                                 "compl"
                                                                                                                 "ete "
                                                                                                                 "did "
                                                                                                                 "not "
                                                                                                                 "succe"
                                                                                                                 "ed";
  auto result = response_future.get();
  ASSERT_TRUE(result) << "result is nullptr after completion";
  EXPECT_EQ(result->code, rclcpp_action::ResultCode::SUCCEEDED) << "ActionWrapper did not succeed";
  ASSERT_EQ(result->result->result, 13) << "Result has unexpected value";
}

TEST_F(TestFixture, SendFeedback)
{
  goal.request = 4;
  ActionClientWrapper<ActionType>::SendGoalOptions send_goal_options;
  send_goal_options.feedback_callback = [this](ActionClientWrapper<ActionType>::ClientGoalHandle::SharedPtr,
                                               const std::shared_ptr<const ActionClientWrapper<ActionType>::Feedback>) {
    action_server_ptr->feeback_received = true;
  };

  auto response_future = action_client_ptr->syncSendGoal(goal, send_goal_options);
  ASSERT_NE(action_client_ptr->getGoalStatus(response_future), ActionGoalStatus::REJECTED) << "Goal was rejected";
  ASSERT_EQ(rclcpp::spin_until_future_complete(node_ptr, response_future), rclcpp::FutureReturnCode::SUCCESS) << "spin_"
                                                                                                                 "until"
                                                                                                                 "_"
                                                                                                                 "futur"
                                                                                                                 "e_"
                                                                                                                 "compl"
                                                                                                                 "ete "
                                                                                                                 "did "
                                                                                                                 "not "
                                                                                                                 "succe"
                                                                                                                 "ed";
  auto result = response_future.get();
  ASSERT_TRUE(result) << "result is nullptr after completion";
  EXPECT_EQ(result->code, rclcpp_action::ResultCode::SUCCEEDED) << "ActionWrapper did not succeed";
  ASSERT_TRUE(action_client_ptr->getFeedback()) << "No feedback available";
  ASSERT_EQ(action_client_ptr->getFeedback()->feedback, 1) << "Feedback has unexpected value";
}

TEST_F(TestFixture, CancelGoalReject)
{
  goal.request = 5;

  auto response_future = action_client_ptr->syncSendGoal(goal);
  ASSERT_NE(action_client_ptr->getGoalStatus(response_future), ActionGoalStatus::REJECTED) << "Goal was rejected";
  ASSERT_FALSE(action_client_ptr->syncCancelLastGoal()) << "Cancellation was not rejected";
  ASSERT_EQ(action_client_ptr->getGoalStatus(response_future), ActionGoalStatus::RUNNING) << "Goal is not running "
                                                                                             "after "
                                                                                             "cancellation was "
                                                                                             "rejected";
}

TEST_F(TestFixture, CancelGoalAccept)
{
  goal.request = 6;

  auto response_future = action_client_ptr->syncSendGoal(goal);
  ASSERT_NE(action_client_ptr->getGoalStatus(response_future), ActionGoalStatus::REJECTED) << "Goal was rejected";
  ASSERT_TRUE(action_client_ptr->syncCancelLastGoal()) << "Cancellation was rejected";
  ASSERT_EQ(rclcpp::spin_until_future_complete(node_ptr, response_future), rclcpp::FutureReturnCode::SUCCESS) << "spin_"
                                                                                                                 "until"
                                                                                                                 "_"
                                                                                                                 "futur"
                                                                                                                 "e_"
                                                                                                                 "compl"
                                                                                                                 "ete "
                                                                                                                 "did "
                                                                                                                 "not "
                                                                                                                 "succe"
                                                                                                                 "ed";
  auto result = response_future.get();
  ASSERT_TRUE(result) << "result is nullptr after completion";
  EXPECT_EQ(result->code, rclcpp_action::ResultCode::CANCELED) << "ActionWrapper was not cancelled";
  ASSERT_EQ(result->result->result, 23) << "Result has unexpected value";
}

TEST_F(TestFixture, ExecuteGoalAbort)
{
  goal.request = 7;

  auto response_future = action_client_ptr->syncSendGoal(goal);
  ASSERT_NE(action_client_ptr->getGoalStatus(response_future), ActionGoalStatus::REJECTED) << "Goal was rejected";
  ASSERT_EQ(rclcpp::spin_until_future_complete(node_ptr, response_future), rclcpp::FutureReturnCode::SUCCESS) << "spin_"
                                                                                                                 "until"
                                                                                                                 "_"
                                                                                                                 "futur"
                                                                                                                 "e_"
                                                                                                                 "compl"
                                                                                                                 "ete "
                                                                                                                 "did "
                                                                                                                 "not "
                                                                                                                 "succe"
                                                                                                                 "ed";
  auto result = response_future.get();
  ASSERT_TRUE(result) << "result is nullptr after completion";
  EXPECT_EQ(result->code, rclcpp_action::ResultCode::ABORTED) << "ActionWrapper did not abort";
  ASSERT_EQ(result->result->result, 30) << "Result has unexpected value";
}

TEST_F(TestFixture, CancelGoalAbort)
{
  goal.request = 8;

  auto response_future = action_client_ptr->syncSendGoal(goal);
  ASSERT_NE(action_client_ptr->getGoalStatus(response_future), ActionGoalStatus::REJECTED) << "Goal was rejected";
  ASSERT_TRUE(action_client_ptr->syncCancelLastGoal()) << "Cancellation was rejected";
  ASSERT_EQ(rclcpp::spin_until_future_complete(node_ptr, response_future), rclcpp::FutureReturnCode::SUCCESS) << "spin_"
                                                                                                                 "until"
                                                                                                                 "_"
                                                                                                                 "futur"
                                                                                                                 "e_"
                                                                                                                 "compl"
                                                                                                                 "ete "
                                                                                                                 "did "
                                                                                                                 "not "
                                                                                                                 "succe"
                                                                                                                 "ed";
  auto result = response_future.get();
  ASSERT_TRUE(result) << "result is nullptr after completion";
  EXPECT_EQ(result->code, rclcpp_action::ResultCode::ABORTED) << "ActionWrapper did not abort";
  ASSERT_EQ(result->result->result, 40) << "Result has unexpected value";
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
