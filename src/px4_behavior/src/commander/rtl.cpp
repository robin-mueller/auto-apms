#include <px4_behavior/commander/mode_executor.hpp>
#include <px4_behavior_interfaces/action/rtl.hpp>

namespace px4_behavior {

class RTLManeuver : public ModeExecutor<px4_behavior_interfaces::action::RTL>
{
   public:
    explicit RTLManeuver(const rclcpp::NodeOptions& options)
        : ModeExecutor{px4_behavior::RTL_MANEUVER_NAME, options, FlightMode::RTL}
    {}

   private:

    // PX4 seems to not always give a completed signal for RTL, so check for disarmed as a fallback completed state
    bool IsCompleted(std::shared_ptr<const Goal> goal_ptr, const px4_msgs::msg::VehicleStatus& vehicle_status)
    {
        return ModeExecutor::IsCompleted(goal_ptr, vehicle_status) ||
               vehicle_status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
    }
};

}  // namespace px4_behavior

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(px4_behavior::RTLManeuver)
