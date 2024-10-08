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

#include "auto_apms/action/test.hpp"
#include "auto_apms/action_client.hpp"
#include "auto_apms/task_base.hpp"
#include "util.hpp"

using namespace std::chrono_literals;
using namespace auto_apms;
using TestAction = auto_apms::action::Test;

class TestTask : public TaskBase<TestAction>
{
   public:
    TestTask(const std::string& name, rclcpp::Node::SharedPtr node_ptr) : TaskBase{name, node_ptr, 0ms, 1ms} {}

    bool OnGoalRequest(std::shared_ptr<const Goal> goal_ptr) override
    {
        if (goal_ptr->request == 1) { return false; }
        return true;
    }

    void SetInitialResult(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr) override
    {
        if (goal_ptr->request == 2) { result_ptr->result = 1; }
        if (goal_ptr->request == 3) { result_ptr->result = 10; }
    }
    bool OnCancelRequest(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr) override
    {
        if (goal_ptr->request == 5) { return false; }
        if (goal_ptr->request == 7) { result_ptr->result = 20; }
        return true;
    }
    TaskStatus CancelGoal(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr) override
    {
        switch (goal_ptr->request) {
            case 6:
                // CancelGoalReject
                if (++result_ptr->result < 23) { return TaskStatus::RUNNING; }
                else {
                    return TaskStatus::SUCCESS;
                }
            case 8:
                // CancelGoalAbort
                result_ptr->result = 40;
                return TaskStatus::FAILURE;
            default:
                throw std::logic_error("Unexpected goal request in CancelGoal: " + std::to_string(goal_ptr->request));
        }
    }
    TaskStatus ExecuteGoal(std::shared_ptr<const Goal> goal_ptr,
                           std::shared_ptr<Feedback> feedback_ptr,
                           std::shared_ptr<Result> result_ptr) override
    {
        switch (goal_ptr->request) {
            case 2:
                // InitialResult
                return TaskStatus::SUCCESS;
            case 3:
                // ExecuteGoal
                if (++result_ptr->result < 13) { return TaskStatus::RUNNING; }
                else {
                    return TaskStatus::SUCCESS;
                }
            case 4:
                // SendFeedback
                feedback_ptr->feedback = 1;
                return feeback_received ? TaskStatus::SUCCESS : TaskStatus::RUNNING;
            case 5:
                // CancelGoalReject
                return TaskStatus::RUNNING;
            case 6:
                // CancelGoalAccept
                return TaskStatus::RUNNING;
            case 7:
                // ExecuteGoalAbort
                result_ptr->result = 30;
                return TaskStatus::FAILURE;
            case 8:
                // CancelGoalAbort
                return TaskStatus::RUNNING;
            default:
                throw std::logic_error("Unexpected goal request in ExecuteGoal: " + std::to_string(goal_ptr->request));
        }
    }

    bool feeback_received = false;
};

class TaskTest : public testing::Test
{
   protected:
    TaskTest()
    {
        node = CreateTestNode();
        test_task = std::make_shared<TestTask>(task_name, node);
        task_client = std::make_shared<ActionClientWrapper<TestAction>>(*node, task_name);
    }

    const std::string task_name = "test_task";
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<TestTask> test_task;
    TestTask::Goal goal;
    std::shared_ptr<ActionClientWrapper<TestAction>> task_client;
};

TEST_F(TaskTest, RejectGoal)
{
    goal.request = 1;

    auto response_future = task_client->SyncSendGoal(goal);
    ASSERT_EQ(task_client->GetGoalStatus(response_future), ActionGoalStatus::REJECTED) << "Goal was not rejected";
}

TEST_F(TaskTest, InitialResult)
{
    goal.request = 2;

    auto response_future = task_client->SyncSendGoal(goal);
    ASSERT_NE(task_client->GetGoalStatus(response_future), ActionGoalStatus::REJECTED) << "Goal was rejected";
    ASSERT_EQ(rclcpp::spin_until_future_complete(node, response_future), rclcpp::FutureReturnCode::SUCCESS)
        << "spin_until_future_complete did not succeed";
    auto result = response_future.get();
    ASSERT_TRUE(result) << "result is nullptr after completion";
    EXPECT_EQ(result->code, rclcpp_action::ResultCode::SUCCEEDED) << "Task did not succeed";
    ASSERT_EQ(result->result->result, 1) << "Result has unexpected value";
}

TEST_F(TaskTest, ExecuteGoal)
{
    goal.request = 3;

    auto response_future = task_client->SyncSendGoal(goal);
    ASSERT_NE(task_client->GetGoalStatus(response_future), ActionGoalStatus::REJECTED) << "Goal was rejected";
    ASSERT_EQ(rclcpp::spin_until_future_complete(node, response_future), rclcpp::FutureReturnCode::SUCCESS)
        << "spin_until_future_complete did not succeed";
    auto result = response_future.get();
    ASSERT_TRUE(result) << "result is nullptr after completion";
    EXPECT_EQ(result->code, rclcpp_action::ResultCode::SUCCEEDED) << "Task did not succeed";
    ASSERT_EQ(result->result->result, 13) << "Result has unexpected value";
}

TEST_F(TaskTest, SendFeedback)
{
    goal.request = 4;
    ActionClientWrapper<TestAction>::SendGoalOptions send_goal_options;
    send_goal_options.feedback_callback = [this](
                                              ActionClientWrapper<TestAction>::ClientGoalHandle::SharedPtr,
                                              const std::shared_ptr<const ActionClientWrapper<TestAction>::Feedback>) {
        test_task->feeback_received = true;
    };

    auto response_future = task_client->SyncSendGoal(goal, send_goal_options);
    ASSERT_NE(task_client->GetGoalStatus(response_future), ActionGoalStatus::REJECTED) << "Goal was rejected";
    ASSERT_EQ(rclcpp::spin_until_future_complete(node, response_future), rclcpp::FutureReturnCode::SUCCESS)
        << "spin_until_future_complete did not succeed";
    auto result = response_future.get();
    ASSERT_TRUE(result) << "result is nullptr after completion";
    EXPECT_EQ(result->code, rclcpp_action::ResultCode::SUCCEEDED) << "Task did not succeed";
    ASSERT_TRUE(task_client->feedback()) << "No feedback available";
    ASSERT_EQ(task_client->feedback()->feedback, 1) << "Feedback has unexpected value";
}

TEST_F(TaskTest, CancelGoalReject)
{
    goal.request = 5;

    auto response_future = task_client->SyncSendGoal(goal);
    ASSERT_NE(task_client->GetGoalStatus(response_future), ActionGoalStatus::REJECTED) << "Goal was rejected";
    ASSERT_FALSE(task_client->SyncCancelLastGoal()) << "Cancellation was not rejected";
    ASSERT_EQ(task_client->GetGoalStatus(response_future), ActionGoalStatus::RUNNING)
        << "Goal is not running after cancellation was rejected";
}

TEST_F(TaskTest, CancelGoalAccept)
{
    goal.request = 6;

    auto response_future = task_client->SyncSendGoal(goal);
    ASSERT_NE(task_client->GetGoalStatus(response_future), ActionGoalStatus::REJECTED) << "Goal was rejected";
    ASSERT_TRUE(task_client->SyncCancelLastGoal()) << "Cancellation was rejected";
    ASSERT_EQ(rclcpp::spin_until_future_complete(node, response_future), rclcpp::FutureReturnCode::SUCCESS)
        << "spin_until_future_complete did not succeed";
    auto result = response_future.get();
    ASSERT_TRUE(result) << "result is nullptr after completion";
    EXPECT_EQ(result->code, rclcpp_action::ResultCode::CANCELED) << "Task was not cancelled";
    ASSERT_EQ(result->result->result, 23) << "Result has unexpected value";
}

TEST_F(TaskTest, ExecuteGoalAbort)
{
    goal.request = 7;

    auto response_future = task_client->SyncSendGoal(goal);
    ASSERT_NE(task_client->GetGoalStatus(response_future), ActionGoalStatus::REJECTED) << "Goal was rejected";
    ASSERT_EQ(rclcpp::spin_until_future_complete(node, response_future), rclcpp::FutureReturnCode::SUCCESS)
        << "spin_until_future_complete did not succeed";
    auto result = response_future.get();
    ASSERT_TRUE(result) << "result is nullptr after completion";
    EXPECT_EQ(result->code, rclcpp_action::ResultCode::ABORTED) << "Task did not abort";
    ASSERT_EQ(result->result->result, 30) << "Result has unexpected value";
}

TEST_F(TaskTest, CancelGoalAbort)
{
    goal.request = 8;

    auto response_future = task_client->SyncSendGoal(goal);
    ASSERT_NE(task_client->GetGoalStatus(response_future), ActionGoalStatus::REJECTED) << "Goal was rejected";
    ASSERT_TRUE(task_client->SyncCancelLastGoal()) << "Cancellation was rejected";
    ASSERT_EQ(rclcpp::spin_until_future_complete(node, response_future), rclcpp::FutureReturnCode::SUCCESS)
        << "spin_until_future_complete did not succeed";
    auto result = response_future.get();
    ASSERT_TRUE(result) << "result is nullptr after completion";
    EXPECT_EQ(result->code, rclcpp_action::ResultCode::ABORTED) << "Task did not abort";
    ASSERT_EQ(result->result->result, 40) << "Result has unexpected value";
}
