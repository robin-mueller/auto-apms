#include <px4_behavior/maneuver/flight_mode_executor.hpp>
#include <px4_behavior_interfaces/action/mission.hpp>

namespace px4_behavior {

class MissionManeuver : public FlightModeExecutor<px4_behavior_interfaces::action::Mission>
{
   public:
    explicit MissionManeuver(const rclcpp::NodeOptions& options)
        : FlightModeExecutor{px4_behavior::MISSION_MANEUVER_NAME, options, FlightMode::Mission}
    {}

   private:
    bool SendActivationCommand(const VehicleCommandClient& client, std::shared_ptr<const Goal> goal_ptr) final
    {
        if (goal_ptr->do_restart) { return client.StartMission(); }
        return client.SyncActivateFlightMode(FlightMode::Mission);
    }
};

}  // namespace px4_behavior

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(px4_behavior::MissionManeuver)
