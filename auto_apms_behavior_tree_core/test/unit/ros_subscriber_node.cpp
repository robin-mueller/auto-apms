// Copyright 2025 Robin Müller
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

#include "auto_apms_behavior_tree_core/node/ros_subscriber_node.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "example_interfaces/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace auto_apms_behavior_tree::core;
using namespace std::chrono_literals;

// Test message type
using TestMessageT = example_interfaces::msg::String;

// Concrete test implementation of RosSubscriberNode
class TestRosSubscriberNode : public RosSubscriberNode<TestMessageT>
{
public:
  TestRosSubscriberNode(
    const std::string & instance_name, const Config & config, RosNodeContext context, const rclcpp::QoS & qos = {10})
  : RosSubscriberNode<TestMessageT>(instance_name, config, context, qos)
  {
  }

  BT::NodeStatus onMessageReceived(const TestMessageT & msg) override
  {
    last_received_message_ = msg.data;
    return BT::NodeStatus::SUCCESS;
  }

  std::string getLastReceivedMessage() const { return last_received_message_; }

  // Expose the tick method for testing
  BT::NodeStatus executeTick() { return tick(); }

private:
  std::string last_received_message_;
};

class RosSubscriberNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("test_subscriber_node");
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    // Create publisher for testing
    publisher_ = node_->create_publisher<TestMessageT>("/test_topic", 10);

    // Create a simple context for testing
    options_.connection = "/test_topic";

    // Create config
    config_.blackboard = BT::Blackboard::create();
  }

  void TearDown() override { rclcpp::shutdown(); }

  RosNodeContext createContext() { return RosNodeContext(node_, callback_group_, executor_, options_); }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::Publisher<TestMessageT>::SharedPtr publisher_;
  NodeRegistrationOptions options_;
  BT::NodeConfig config_;
};

TEST_F(RosSubscriberNodeTest, NodeCreation)
{
  // Test that we can create a RosSubscriberNode without errors
  EXPECT_NO_THROW({ TestRosSubscriberNode subscriber_node("test_subscriber", config_, createContext()); });
}

TEST_F(RosSubscriberNodeTest, TopicNameRetrieval)
{
  TestRosSubscriberNode subscriber_node("test_subscriber", config_, createContext());

  // The topic name should be retrieved from the context
  EXPECT_EQ(subscriber_node.getTopicName(), "/test_topic");
}

TEST_F(RosSubscriberNodeTest, SubscriberRegistry)
{
  // Create two subscriber nodes with the same topic
  TestRosSubscriberNode subscriber_node1("test_subscriber1", config_, createContext());
  TestRosSubscriberNode subscriber_node2("test_subscriber2", config_, createContext());

  // Both should have the same topic name (shared subscriber)
  EXPECT_EQ(subscriber_node1.getTopicName(), subscriber_node2.getTopicName());
}

TEST_F(RosSubscriberNodeTest, InitialTickState)
{
  TestRosSubscriberNode subscriber_node("test_subscriber", config_, createContext());

  // Initial tick should return FAILURE (no message received yet)
  BT::NodeStatus status = subscriber_node.executeTick();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(RosSubscriberNodeTest, MessageReception)
{
  TestRosSubscriberNode subscriber_node("test_subscriber", config_, createContext());

  // Add node to executor
  executor_->add_node(node_);

  // Start executor in background thread
  std::thread executor_thread([this]() { executor_->spin(); });

  // Publish a test message
  TestMessageT test_msg;
  test_msg.data = "Hello, ROS!";

  // Give some time for subscription setup
  std::this_thread::sleep_for(100ms);

  publisher_->publish(test_msg);

  // Give time for message processing
  std::this_thread::sleep_for(100ms);

  // Stop executor
  executor_->cancel();
  executor_thread.join();

  // Tick the subscriber node
  BT::NodeStatus status = subscriber_node.executeTick();

  // Should succeed and receive the message
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(subscriber_node.getLastReceivedMessage(), "Hello, ROS!");
}

TEST_F(RosSubscriberNodeTest, MultipleSubscribersSameTopicSharing)
{
  // Create two subscriber nodes for the same topic
  TestRosSubscriberNode subscriber_node1("test_subscriber1", config_, createContext());
  TestRosSubscriberNode subscriber_node2("test_subscriber2", config_, createContext());

  // Add node to executor
  executor_->add_node(node_);

  // Start executor in background thread
  std::thread executor_thread([this]() { executor_->spin(); });

  // Publish a test message
  TestMessageT test_msg;
  test_msg.data = "Shared message";

  // Give some time for subscription setup
  std::this_thread::sleep_for(100ms);

  publisher_->publish(test_msg);

  // Give time for message processing
  std::this_thread::sleep_for(100ms);

  // Stop executor
  executor_->cancel();
  executor_thread.join();

  // Both nodes should receive the same message
  EXPECT_EQ(subscriber_node1.executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(subscriber_node2.executeTick(), BT::NodeStatus::SUCCESS);

  EXPECT_EQ(subscriber_node1.getLastReceivedMessage(), "Shared message");
  EXPECT_EQ(subscriber_node2.getLastReceivedMessage(), "Shared message");
}

TEST_F(RosSubscriberNodeTest, SubscriberInstanceLifecycle)
{
  {
    // Create a subscriber node in limited scope
    TestRosSubscriberNode subscriber_node("test_subscriber", config_, createContext());
    EXPECT_EQ(subscriber_node.getTopicName(), "/test_topic");
  }
  // Node goes out of scope and should clean up properly

  // Create another node with the same topic - should work fine
  TestRosSubscriberNode new_subscriber_node("new_test_subscriber", config_, createContext());
  EXPECT_EQ(new_subscriber_node.getTopicName(), "/test_topic");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
