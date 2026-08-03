#include "vehicle_command_authorization.hpp"

#include <gtest/gtest.h>

namespace px4_ctrl_ros2
{

TEST(VehicleCommandAuthorization, OffboardDisabledBlocksAllCommands)
{
  EXPECT_FALSE(vehicle_command_allowed(VehicleCommandAuthority::OFFBOARD, false, false));
  EXPECT_FALSE(vehicle_command_allowed(VehicleCommandAuthority::ARM_DISARM, false, false));
  EXPECT_FALSE(vehicle_command_allowed(VehicleCommandAuthority::OFFBOARD, false, true));
  EXPECT_FALSE(vehicle_command_allowed(VehicleCommandAuthority::ARM_DISARM, false, true));
}

TEST(VehicleCommandAuthorization, AutoArmDisabledOnlyAllowsOffboardCommands)
{
  EXPECT_TRUE(vehicle_command_allowed(VehicleCommandAuthority::OFFBOARD, true, false));
  EXPECT_FALSE(vehicle_command_allowed(VehicleCommandAuthority::ARM_DISARM, true, false));
}

TEST(VehicleCommandAuthorization, BothPermissionsAllowAllCommands)
{
  EXPECT_TRUE(vehicle_command_allowed(VehicleCommandAuthority::OFFBOARD, true, true));
  EXPECT_TRUE(vehicle_command_allowed(VehicleCommandAuthority::ARM_DISARM, true, true));
}

}  // namespace px4_ctrl_ros2
