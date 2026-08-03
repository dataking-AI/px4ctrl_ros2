#pragma once

namespace px4_ctrl_ros2
{

enum class VehicleCommandAuthority
{
  OFFBOARD,
  ARM_DISARM
};

constexpr bool vehicle_command_allowed(
  VehicleCommandAuthority authority,
  bool enable_offboard_command,
  bool enable_auto_arm)
{
  if (!enable_offboard_command) {
    return false;
  }
  return authority != VehicleCommandAuthority::ARM_DISARM || enable_auto_arm;
}

}  // namespace px4_ctrl_ros2
