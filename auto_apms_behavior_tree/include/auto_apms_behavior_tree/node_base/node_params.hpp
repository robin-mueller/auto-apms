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

#pragma once

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace auto_apms_behavior_tree {

struct RosNodeParams
{
    std::weak_ptr<rclcpp::Node> nh;

    /**
     * @brief Default port value of ROS 2 communication interfaces.
     *
     * This has different meaning based on the context:
     * - RosActionNode: name of the action server
     * - RosServiceNode: name of the service
     * - RosPublisherNode: name of the topic to publish to
     * - RosSubscriberNode: name of the topic to subscribe to
     */
    std::string default_port_value;

    // parameters used only by service client and action clients

    // timeout when sending a request
    std::chrono::milliseconds server_timeout = std::chrono::milliseconds(1000);
    // timeout used when detecting the server the first time
    std::chrono::milliseconds wait_for_server_timeout = std::chrono::milliseconds(500);
};

}  // namespace auto_apms_behavior_tree
