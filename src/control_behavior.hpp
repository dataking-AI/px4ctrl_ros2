#pragma once

#include <Eigen/Geometry>

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

inline Eigen::Quaterniond align_nav_attitude_to_fcu(
  const Eigen::Quaterniond & fcu_attitude,
  const Eigen::Quaterniond & nav_attitude,
  const Eigen::Quaterniond & desired_attitude)
{
  return (fcu_attitude * nav_attitude.inverse() * desired_attitude).normalized();
}

}  // namespace px4_ctrl_ros2
