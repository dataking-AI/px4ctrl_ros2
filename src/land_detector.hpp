#pragma once

#include "flight_state.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>

namespace px4_ctrl_ros2
{

// Port of PX4CtrlFSM::land_detector(): touchdown is inferred from the
// controller asking for a lower position while the drone stays still, so no
// world-frame ground height is needed.
class LandDetector
{
public:
  // Constraint 1: the desired position is this far below the estimated one.
  static constexpr double kPositionDeviation = -0.5;
  // Constraint 2: the estimated speed stays below this value.
  static constexpr double kVelocityThreshold = 0.1;
  // Constraint 3: constraints 1 and 2 hold for this long.
  static constexpr double kHoldSeconds = 3.0;

  // Assumed on the ground until a flight state transition clears it.
  bool landed{true};

  void update(
    FlightState previous_state,
    FlightState state,
    bool armed,
    double desired_z,
    double odom_z,
    double odom_speed,
    const rclcpp::Time & now)
  {
    if (previous_state == FlightState::MANUAL_CTRL &&
      (state == FlightState::AUTO_HOVER || state == FlightState::AUTO_TAKEOFF))
    {
      // Taking over from the pilot always starts airborne.
      landed = false;
    }

    if (state == FlightState::MANUAL_CTRL && !armed) {
      landed = true;
      reset_condition();
      return;
    }

    if (landed) {
      reset_condition();
      return;
    }

    const bool condition_met =
      std::isfinite(desired_z) && std::isfinite(odom_z) && std::isfinite(odom_speed) &&
      (desired_z - odom_z) < kPositionDeviation && odom_speed < kVelocityThreshold;

    if (!condition_met) {
      reset_condition();
      return;
    }

    if (!condition_started_) {
      condition_started_ = true;
      condition_start_time_ = now;
      return;
    }

    if ((now - condition_start_time_).seconds() > kHoldSeconds) {
      landed = true;
    }
  }

  void reset_condition()
  {
    condition_started_ = false;
  }

  bool condition_started() const
  {
    return condition_started_;
  }

private:
  bool condition_started_{false};
  rclcpp::Time condition_start_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace px4_ctrl_ros2
