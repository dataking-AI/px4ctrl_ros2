#pragma once

namespace px4_ctrl_ros2
{

enum class FlightState
{
  MANUAL_CTRL,
  AUTO_HOVER,
  CMD_CTRL,
  AUTO_TAKEOFF,
  AUTO_LAND,
  EXITING_OFFBOARD,
  FAILSAFE
};

constexpr bool should_reset_thrust_mapping(FlightState from, FlightState to)
{
  return from == FlightState::MANUAL_CTRL &&
         (to == FlightState::AUTO_HOVER || to == FlightState::AUTO_TAKEOFF);
}

}  // namespace px4_ctrl_ros2
