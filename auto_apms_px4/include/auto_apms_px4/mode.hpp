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

#include "auto_apms_core/action_context.hpp"
#include "px4_ros2/components/mode.hpp"
#include "px4_ros2/odometry/attitude.hpp"
#include "px4_ros2/odometry/global_position.hpp"
#include "px4_ros2/odometry/local_position.hpp"
#include "px4_ros2/utils/geodesic.hpp"
#include "px4_ros2/utils/geometry.hpp"

namespace auto_apms_px4
{

/**
 * @brief Class to model the mode referring to an external task.
 */
template <class ActionT>
class ModeBase : public px4_ros2::ModeBase
{
protected:
  using ActionContextType = auto_apms_core::ActionContext<ActionT>;
  using Goal = typename ActionContextType::Goal;
  using Result = typename ActionContextType::Result;
  using Feedback = typename ActionContextType::Feedback;

  ModeBase(rclcpp::Node& node, const Settings& settings, const std::string& topic_namespace_prefix,
           std::shared_ptr<ActionContextType> action_context_ptr)
    : px4_ros2::ModeBase{ node, settings, topic_namespace_prefix }, action_context_ptr_{ action_context_ptr }
  {
  }

private:
  virtual void OnActivateWithGoal(std::shared_ptr<const Goal> goal_ptr);
  virtual void UpdateSetpointWithGoal(float dt_s, std::shared_ptr<const Goal> goal_ptr,
                                      std::shared_ptr<Feedback> feedback_ptr, std::shared_ptr<Result> result_ptr) = 0;
  virtual void onDeactivate() override
  {
  }

  void onActivate() override;
  void updateSetpoint(float dt_s) override;

  const std::shared_ptr<ActionContextType> action_context_ptr_;
};

template <class ActionT>
void ModeBase<ActionT>::OnActivateWithGoal(std::shared_ptr<const Goal> goal_ptr)
{
  (void)goal_ptr;
}

template <class ActionT>
void ModeBase<ActionT>::onActivate()
{
  OnActivateWithGoal(action_context_ptr_->getGoalHandlePtr()->get_goal());
}

template <class ActionT>
void ModeBase<ActionT>::updateSetpoint(float dt_s)
{
  UpdateSetpointWithGoal(dt_s, action_context_ptr_->getGoalHandlePtr()->get_goal(),
                         action_context_ptr_->getFeedbackPtr(), action_context_ptr_->getResultPtr());
}

template <class ActionT>
class PositionAwareMode : public ModeBase<ActionT>
{
protected:
  using typename ModeBase<ActionT>::ActionContextType;

  PositionAwareMode(rclcpp::Node& node, const px4_ros2::ModeBase::Settings& settings,
                    const std::string& topic_namespace_prefix, std::shared_ptr<ActionContextType> action_context_ptr)
    : ModeBase<ActionT>{ node, settings, topic_namespace_prefix, action_context_ptr }
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

}  // namespace auto_apms_px4
