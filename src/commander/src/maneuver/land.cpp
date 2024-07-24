#include <commander/maneuver/flight_mode_executor.hpp>
#include <commander_interfaces/action/land.hpp>

namespace commander {

class LandManeuver : public FlightModeExecutor<commander_interfaces::action::Land>
{
   public:
    explicit LandManeuver(const rclcpp::NodeOptions& options)
        : FlightModeExecutor{commander::LAND_MANEUVER_NAME, options, FlightMode::Land}
    {}

   private:
    bool SendActivationCommand(const VehicleCommandClient& client, std::shared_ptr<const Goal> goal_ptr)
    {
        (void)goal_ptr;
        return client.Land();
    }
};

}  // namespace commander

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(commander::LandManeuver)
