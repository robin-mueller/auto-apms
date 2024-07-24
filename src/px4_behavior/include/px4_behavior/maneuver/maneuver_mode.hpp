#pragma once

#include <px4_behavior/maneuver/action_context.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geodesic.hpp>
#include <px4_ros2/utils/geometry.hpp>

namespace px4_behavior {

/**
 * \brief Class to model a external maneuver mode.
 */
template <class ActionT>
class ManeuverMode : public px4_ros2::ModeBase
{
   protected:
    using Goal = typename ActionContext<ActionT>::Goal;
    using Result = typename ActionContext<ActionT>::Result;
    using Feedback = typename ActionContext<ActionT>::Feedback;
    using ActionContextType = ActionContext<ActionT>;

    ManeuverMode(rclcpp::Node& node,
                 const Settings& settings,
                 const std::string& topic_namespace_prefix,
                 std::shared_ptr<ActionContextType> action_context_ptr)
        : ModeBase{node, settings, topic_namespace_prefix}, action_context_ptr_{action_context_ptr}
    {}

   private:
    virtual void OnActivateWithGoal(std::shared_ptr<const Goal> goal_ptr);
    virtual void UpdateSetpointWithGoal(float dt_s,
                                        std::shared_ptr<const Goal> goal_ptr,
                                        std::shared_ptr<Feedback> feedback_ptr,
                                        std::shared_ptr<Result> result_ptr) = 0;
    virtual void onDeactivate() override {}

    void onActivate() override;
    void updateSetpoint(float dt_s) override;

    const std::shared_ptr<ActionContext<ActionT>> action_context_ptr_;
};

template <class ActionT>
void ManeuverMode<ActionT>::OnActivateWithGoal(std::shared_ptr<const Goal> goal_ptr)
{
    (void)goal_ptr;
}

template <class ActionT>
void ManeuverMode<ActionT>::onActivate()
{
    OnActivateWithGoal(action_context_ptr_->goal_handle()->get_goal());
}

template <class ActionT>
void ManeuverMode<ActionT>::updateSetpoint(float dt_s)
{
    UpdateSetpointWithGoal(dt_s,
                           action_context_ptr_->goal_handle()->get_goal(),
                           action_context_ptr_->feedback(),
                           action_context_ptr_->result());
}

template <class ActionT>
class PositionAwareMode : public ManeuverMode<ActionT>
{
   protected:
    using ActionContextType = ActionContext<ActionT>;

    PositionAwareMode(rclcpp::Node& node,
                      const px4_ros2::ModeBase::Settings& settings,
                      const std::string& topic_namespace_prefix,
                      std::shared_ptr<ActionContextType> action_context_ptr)
        : ManeuverMode<ActionT>{node, settings, topic_namespace_prefix, action_context_ptr}
    {
        vehicle_global_position_ptr_ = std::make_shared<px4_ros2::OdometryGlobalPosition>(*this);
        vehicle_local_position_ptr_ = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
        vehicle_attitude_ptr_ = std::make_shared<px4_ros2::OdometryAttitude>(*this);
    }

    bool IsPositionReached(const Eigen::Vector3d& target_position_f_glob) const;
    bool IsAltitudeReached(float target_altitude_amsl_m) const;
    bool IsHeadingReached(float target_heading_rad) const;

   protected:
    std::shared_ptr<px4_ros2::OdometryGlobalPosition> vehicle_global_position_ptr_;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> vehicle_local_position_ptr_;
    std::shared_ptr<px4_ros2::OdometryAttitude> vehicle_attitude_ptr_;
};

template <class ActionT>
bool PositionAwareMode<ActionT>::IsPositionReached(const Eigen::Vector3d& target_position_f_glob) const
{
    static constexpr float kPositionErrorThresholdMeter = 0.5f;           // [m]
    static constexpr float kVelocityErrorThresholdMeterPerSecond = 0.3f;  // [m/s]
    const float position_error_m =
        px4_ros2::distanceToGlobalPosition(vehicle_global_position_ptr_->position(), target_position_f_glob);
    return (position_error_m < kPositionErrorThresholdMeter) &&
           (vehicle_local_position_ptr_->velocityNed().norm() < kVelocityErrorThresholdMeterPerSecond);
}

template <class ActionT>
bool PositionAwareMode<ActionT>::IsAltitudeReached(float target_altitude_amsl_m) const
{
    auto target_position_f_glob = vehicle_global_position_ptr_->position();
    target_position_f_glob.z() = target_altitude_amsl_m;
    return IsPositionReached(target_position_f_glob);
}

template <class ActionT>
bool PositionAwareMode<ActionT>::IsHeadingReached(float target_heading_rad) const
{
    using namespace px4_ros2::literals;
    static constexpr float kHeadingErrorThresholdRad = 7.0_deg;
    const float heading_error_wrapped = px4_ros2::wrapPi(target_heading_rad - vehicle_attitude_ptr_->yaw());
    return fabsf(heading_error_wrapped) < kHeadingErrorThresholdRad;
}

}  // namespace px4_behavior
