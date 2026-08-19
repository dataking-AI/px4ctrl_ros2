#pragma once

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

}  // namespace px4_ctrl_ros2
