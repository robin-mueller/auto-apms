#include <commander/maneuver/flight_mode_executor.hpp>
#include <commander_interfaces/action/enable_hold.hpp>

namespace commander {

class EnableHoldManeuver : public FlightModeExecutor<commander_interfaces::action::EnableHold>
{
   public:
    explicit EnableHoldManeuver(const rclcpp::NodeOptions& options)
        : FlightModeExecutor{
              commander::ENABLE_HOLD_MANEUVER_NAME, options, FlightMode::Hold, false}
    {}

   private:
    bool IsCompleted(std::shared_ptr<const Goal> goal_ptr, const px4_msgs::msg::VehicleStatus& vehicle_status) final
    {
        (void)goal_ptr;
        (void)vehicle_status;
        // For HOLD mode there is no completion signal. Once activated it is considered complete.
        return true;
    }
};

}  // namespace commander

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(commander::EnableHoldManeuver)
