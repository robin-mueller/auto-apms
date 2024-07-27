#include <px4_behavior/commander/task.hpp>
#include <px4_behavior/commander/vehicle_command_client.hpp>
#include <px4_behavior_interfaces/action/arm_disarm.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

namespace px4_behavior {

class ArmDisarmManeuver : public TaskBase<px4_behavior_interfaces::action::ArmDisarm>
{
    enum State { WAIT_FOR_READY_TO_ARM, SEND_COMMAND, WAIT_FOR_ARMING_STATE_REACHED };

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_ptr_;
    State state_;
    bool ready_to_arm_{false};
    bool is_armed_{false};
    std::function<bool()> is_arming_state_reached_check__;
    std::function<bool()> send_command_callback_;
    const VehicleCommandClient vehicle_command_client_;

   public:
    explicit ArmDisarmManeuver(const rclcpp::NodeOptions& options)
        : TaskBase{px4_behavior::ARM_DISARM_MANEUVER_NAME, options}, vehicle_command_client_{*this->node_ptr_}
    {
        vehicle_status_sub_ptr_ = this->node_ptr_->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status",
            rclcpp::QoS(1).best_effort(),
            [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) {
                is_armed_ = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
                ready_to_arm_ = msg->pre_flight_checks_pass;
            });
    }

   private:
    bool OnGoalRequest(std::shared_ptr<const Goal> goal_ptr) final
    {
        // Reject goal if goal wants to arm, but UAV is not ready to arm and it is not requested to wait for ready to
        // arm
        if (goal_ptr->arming_state == Goal::ARMING_STATE_ARM && !goal_ptr->wait_until_ready_to_arm && !ready_to_arm_) {
            RCLCPP_ERROR(this->node_ptr_->get_logger(),
                         "UAV can currently not be armed and wait_until_ready_to_arm is false.");
            return false;
        }

        if (goal_ptr->arming_state == Goal::ARMING_STATE_ARM) {
            // If goal is to arm
            state_ = ready_to_arm_ ? SEND_COMMAND : WAIT_FOR_READY_TO_ARM;
            is_arming_state_reached_check__ = [this]() { return is_armed_; };
            send_command_callback_ = [this]() { return vehicle_command_client_.Arm(); };
        }
        else {
            // If goal is to disarm
            state_ = SEND_COMMAND;
            is_arming_state_reached_check__ = [this]() { return !is_armed_; };
            send_command_callback_ = [this]() { return vehicle_command_client_.Disarm(); };
        }

        // Succeed directly if arming state is already reached
        if ((goal_ptr->arming_state == Goal::ARMING_STATE_ARM && is_armed_) ||
            (goal_ptr->arming_state == Goal::ARMING_STATE_DISARM && !is_armed_)) {
            state_ = WAIT_FOR_ARMING_STATE_REACHED;
        }

        return true;
    }

    TaskStatus ExecuteGoal(std::shared_ptr<const Goal> goal_ptr,
                                       std::shared_ptr<Feedback> feedback_ptr,
                                       std::shared_ptr<Result> result_ptr) final
    {
        (void)goal_ptr;
        (void)feedback_ptr;

        switch (state_) {
            case WAIT_FOR_READY_TO_ARM:
                if (ready_to_arm_) state_ = SEND_COMMAND;
                break;
            case SEND_COMMAND:
                if (send_command_callback_()) { state_ = WAIT_FOR_ARMING_STATE_REACHED; }
                else {
                    RCLCPP_ERROR(this->node_ptr_->get_logger(), "Couldn't send arm/disarm command. Aborting...");
                    result_ptr->state_changed = false;
                    return TaskStatus::FAILURE;
                }
                break;
            case WAIT_FOR_ARMING_STATE_REACHED:
                if (is_arming_state_reached_check__()) {
                    result_ptr->state_changed = true;
                    return TaskStatus::SUCCESS;
                }
                break;
        }

        return TaskStatus::RUNNING;
    }
};

}  // namespace px4_behavior

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(px4_behavior::ArmDisarmManeuver)
