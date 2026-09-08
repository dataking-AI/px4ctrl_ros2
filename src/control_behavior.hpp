#pragma once

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace px4_ctrl_ros2
{

enum class AutoLandRcAction
{
  CONTINUE_LANDING,
  EXIT_OFFBOARD,
  HOLD_POSITION
};

constexpr AutoLandRcAction auto_land_rc_action(
  bool rc_required,
  bool rc_available,
  bool is_hover_mode,
  bool is_command_mode)
{
  if (!rc_required) {
    return AutoLandRcAction::CONTINUE_LANDING;
  }
  if (!rc_available || !is_hover_mode) {
    return AutoLandRcAction::EXIT_OFFBOARD;
  }
  if (!is_command_mode) {
    return AutoLandRcAction::HOLD_POSITION;
  }
  return AutoLandRcAction::CONTINUE_LANDING;
}

constexpr float attitude_yaw_sp_move_rate()
{
  return 0.0F;
}

// Lowest z the RC stick may command the hover setpoint to. It is a world-frame
// constant like in PX4Ctrl, not a measured ground height, so it never depends
// on where the odometry origin happens to be.
constexpr double kMinHoverZSetpoint = -0.3;

inline double clamp_hover_z_setpoint(double proposed_z)
{
  if (!std::isfinite(proposed_z)) {
    return kMinHoverZSetpoint;
  }
  return std::max(proposed_z, kMinHoverZSetpoint);
}

inline Eigen::Quaterniond align_nav_attitude_to_fcu(
  const Eigen::Quaterniond & fcu_attitude,
  const Eigen::Quaterniond & nav_attitude,
  const Eigen::Quaterniond & desired_attitude)
{
  return (fcu_attitude * nav_attitude.inverse() * desired_attitude).normalized();
}

}  // namespace px4_ctrl_ros2
