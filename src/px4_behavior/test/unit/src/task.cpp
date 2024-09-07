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

#include "px4_behavior/action/test.hpp"
#include "px4_behavior/action_client.hpp"
#include "px4_behavior/task_base.hpp"
#include "util.hpp"

using namespace px4_behavior;
using TestAction = px4_behavior::action::Test;

class TestTask : public TaskBase<TestAction>
{
   public:
    TestTask(const std::string& name, rclcpp::Node::SharedPtr node_ptr) : TaskBase{name, node_ptr} {}

    bool OnGoalRequest(std::shared_ptr<const Goal> goal_ptr) override
    {
        if (goal_ptr->request == 1) { return false; }
        return true;
    }

    void SetDefaultResult(std::shared_ptr<Result> result_ptr) override { result_ptr->result = 1; }
    bool OnCancelRequest(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr) override
    {
        return true;
    }
    TaskStatus CancelGoal(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr) override
    {
        (void)goal_ptr;
        (void)result_ptr;
        return TaskStatus::SUCCESS;
    }
    TaskStatus ExecuteGoal(std::shared_ptr<const Goal> goal_ptr,
                           std::shared_ptr<Feedback> feedback_ptr,
                           std::shared_ptr<Result> result_ptr) override
    {
        (void)goal_ptr;
        (void)feedback_ptr;
        switch (goal_ptr->request) {
            case 2:
                // Default result
                return TaskStatus::SUCCESS;
            case 3:
                result_ptr->result = 2;
                return TaskStatus::SUCCESS;
            default:
                throw std::runtime_error("Illegal goal request: " + std::to_string(goal_ptr->request));
        }
    }
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

    ~TaskTest() override
    {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    void SetUp() override
    {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    void TearDown() override
    {
        // Code here will be called immediately after each test (right
        // before the destructor).
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
    ASSERT_EQ(task_client->GetGoalStatus(response_future), ActionGoalStatus::REJECTED);
}

TEST_F(TaskTest, DefaultResult)
{
    // Default result value
    goal.request = 2;
    auto response_future = task_client->SyncSendGoal(goal);
    ASSERT_EQ(rclcpp::spin_until_future_complete(node, response_future), rclcpp::FutureReturnCode::SUCCESS)
        << "spin_until_future_complete did not succeed";
    auto result = response_future.get();
    ASSERT_TRUE(result) << "result is nullptr after completion";
    ASSERT_EQ(result->code, rclcpp_action::ResultCode::SUCCEEDED) << "Task did not succeed";
    ASSERT_EQ(result->result->result, 1) << "Result has unexpected value";
}

TEST_F(TaskTest, ExecuteGoal)
{
    // Default result value
    goal.request = 3;
    auto response_future = task_client->SyncSendGoal(goal);
    ASSERT_EQ(rclcpp::spin_until_future_complete(node, response_future), rclcpp::FutureReturnCode::SUCCESS)
        << "spin_until_future_complete did not succeed";
    auto result = response_future.get();
    ASSERT_TRUE(result) << "result is nullptr after completion";
    ASSERT_EQ(result->code, rclcpp_action::ResultCode::SUCCEEDED) << "Task did not succeed";
    ASSERT_EQ(result->result->result, 2) << "Result has unexpected value";
}
