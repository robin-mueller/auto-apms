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

#include <auto_apms/bt_executor.hpp>
#include <auto_apms_examples/msg/landing_approach.hpp>
#include <definitions.hpp>

#define KEY_ALTITUDE "altitude_amsl_m"
#define KEY_NEXT_LANDING_SITE_ID "next_landing_site_id"
#define KEY_IS_APPROACHING_LANDING "is_approaching_landing"

using namespace auto_apms;
using LandingApproachMsg = auto_apms_examples::msg::LandingApproach;

namespace auto_apms::ops_engine {

class MissionManagerExecutor : public BTExecutor
{
   public:
    MissionManagerExecutor(const rclcpp::NodeOptions& options);

   private:
    void SetupBehaviorTreeFactory(rclcpp::Node::SharedPtr node_ptr, BT::BehaviorTreeFactory& factory) final;
    void OnTreeCreated(BT::Blackboard& global_blackboard) final;

    BT::Blackboard::Ptr initial_bb_;
    rclcpp::Publisher<LandingApproachMsg>::SharedPtr pub_landing_approach_ptr_;
    rclcpp::TimerBase::SharedPtr publish_timer_ptr_;
};

MissionManagerExecutor::MissionManagerExecutor(const rclcpp::NodeOptions& options)
    : BTExecutor{"mission_manager", options, 6666}, initial_bb_{BT::Blackboard::create()}
{
    initial_bb_->set<double>(KEY_ALTITUDE, 190.0);
    initial_bb_->set<int>(KEY_NEXT_LANDING_SITE_ID, 0);
    initial_bb_->set<bool>(KEY_IS_APPROACHING_LANDING, false);
    initial_bb_->cloneInto(*global_blackboard());

    pub_landing_approach_ptr_ = node()->create_publisher<LandingApproachMsg>(LANDING_APPROCH_TOPIC_NAME, 10);
    publish_timer_ptr_ = node()->create_wall_timer(std::chrono::milliseconds(10), [this]() {
        LandingApproachMsg landing_approach_msg;
        landing_approach_msg.next_landing_site_id = global_blackboard()->get<int>(KEY_NEXT_LANDING_SITE_ID);
        landing_approach_msg.is_approaching = global_blackboard()->get<bool>(KEY_IS_APPROACHING_LANDING);
        pub_landing_approach_ptr_->publish(landing_approach_msg);
    });
}

void MissionManagerExecutor::SetupBehaviorTreeFactory(rclcpp::Node::SharedPtr node_ptr,
                                                      BT::BehaviorTreeFactory& factory)
{}

void MissionManagerExecutor::OnTreeCreated(BT::Blackboard& global_blackboard)
{
    initial_bb_->cloneInto(global_blackboard);  // Reset the global blackboard
}

}  // namespace auto_apms::ops_engine

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms::ops_engine::MissionManagerExecutor);
