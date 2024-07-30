#include <px4_behavior/commander/mode_executor.hpp>
#include <px4_behavior_interfaces/action/mission.hpp>

namespace px4_behavior {

class MissionTask : public ModeExecutor<px4_behavior_interfaces::action::Mission>
{
   public:
    explicit MissionTask(const rclcpp::NodeOptions& options)
        : ModeExecutor{px4_behavior::MISSION_TASK_NAME, options, FlightMode::Mission}
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
RCLCPP_COMPONENTS_REGISTER_NODE(px4_behavior::MissionTask)
