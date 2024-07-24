#include <commander/maneuver/flight_mode_executor.hpp>
#include <commander_interfaces/action/rtl.hpp>

namespace commander {

class RTLManeuver : public FlightModeExecutor<commander_interfaces::action::RTL>
{
   public:
    explicit RTLManeuver(const rclcpp::NodeOptions& options)
        : FlightModeExecutor{commander::RTL_MANEUVER_NAME, options, FlightMode::RTL}
    {}

   private:

    // PX4 seems to not always give a completed signal for RTL, so check for disarmed as a fallback completed state
    bool IsCompleted(std::shared_ptr<const Goal> goal_ptr, const px4_msgs::msg::VehicleStatus& vehicle_status)
    {
        return FlightModeExecutor::IsCompleted(goal_ptr, vehicle_status) ||
               vehicle_status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
    }
};

}  // namespace commander

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(commander::RTLManeuver)
