#include "control_behavior.hpp"
#include "flight_state.hpp"
#include "thrust_estimation_input.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace px4_ctrl_ros2
{

TEST(ThrustMappingResetPolicy, ResetsOnlyWhenManualControlEntersAutomaticControl)
{
  EXPECT_TRUE(should_reset_thrust_mapping(FlightState::MANUAL_CTRL, FlightState::AUTO_HOVER));
  EXPECT_TRUE(should_reset_thrust_mapping(FlightState::MANUAL_CTRL, FlightState::AUTO_TAKEOFF));

  EXPECT_FALSE(should_reset_thrust_mapping(FlightState::AUTO_HOVER, FlightState::CMD_CTRL));
  EXPECT_FALSE(should_reset_thrust_mapping(FlightState::CMD_CTRL, FlightState::AUTO_HOVER));
  EXPECT_FALSE(should_reset_thrust_mapping(FlightState::AUTO_TAKEOFF, FlightState::AUTO_HOVER));
  EXPECT_FALSE(should_reset_thrust_mapping(FlightState::AUTO_HOVER, FlightState::AUTO_LAND));
  EXPECT_FALSE(should_reset_thrust_mapping(FlightState::MANUAL_CTRL, FlightState::FAILSAFE));
}

TEST(AutoLandRcPolicy, ExitsOffboardWhenRequiredRcIsUnavailableOrLeavesHoverMode)
{
  EXPECT_EQ(
    AutoLandRcAction::EXIT_OFFBOARD,
    auto_land_rc_action(true, false, true, true));
  EXPECT_EQ(
    AutoLandRcAction::EXIT_OFFBOARD,
    auto_land_rc_action(true, true, false, true));
}

TEST(AutoLandRcPolicy, HoldsPositionWhenCommandModeIsReleased)
{
  EXPECT_EQ(
    AutoLandRcAction::HOLD_POSITION,
    auto_land_rc_action(true, true, true, false));
}

TEST(AutoLandRcPolicy, ContinuesWithAuthorizedRc)
{
  EXPECT_EQ(
    AutoLandRcAction::CONTINUE_LANDING,
    auto_land_rc_action(true, true, true, true));
}

TEST(AutoLandRcPolicy, IgnoresRcStateWhenRcIsNotRequired)
{
  EXPECT_EQ(
    AutoLandRcAction::CONTINUE_LANDING,
    auto_land_rc_action(false, false, false, false));
}

TEST(AttitudeSetpointPolicy, IgnoresYawRateFeedforward)
{
  EXPECT_FLOAT_EQ(0.0F, attitude_yaw_sp_move_rate());
}

TEST(ThrustEstimationInput, ConvertsSensorAccelerationFromFrdToFlu)
{
  const auto acceleration_flu = sensor_acceleration_frd_to_flu({1.0f, 2.0f, -3.0f});

  ASSERT_TRUE(acceleration_flu.has_value());
  EXPECT_DOUBLE_EQ(1.0, acceleration_flu->x());
  EXPECT_DOUBLE_EQ(-2.0, acceleration_flu->y());
  EXPECT_DOUBLE_EQ(3.0, acceleration_flu->z());
}

TEST(ThrustEstimationInput, RejectsNonFiniteSensorAcceleration)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();

  EXPECT_FALSE(sensor_acceleration_frd_to_flu({nan, 0.0f, -9.81f}).has_value());
  EXPECT_FALSE(sensor_acceleration_frd_to_flu({0.0f, infinity, -9.81f}).has_value());
}

TEST(ThrustEstimationInput, ComputesAccelerometerSampleTimestamp)
{
  EXPECT_EQ(1'000'020U, sensor_acceleration_timestamp_us(1'000'000U, 20).value());
  EXPECT_EQ(999'980U, sensor_acceleration_timestamp_us(1'000'000U, -20).value());
}

TEST(ThrustEstimationInput, RejectsInvalidAccelerometerSampleTimestamp)
{
  EXPECT_FALSE(sensor_acceleration_timestamp_us(0, 0).has_value());
  EXPECT_FALSE(
    sensor_acceleration_timestamp_us(
      1'000'000U,
      std::numeric_limits<int32_t>::max()).has_value());
  EXPECT_FALSE(sensor_acceleration_timestamp_us(20U, -20).has_value());
  EXPECT_FALSE(
    sensor_acceleration_timestamp_us(
      std::numeric_limits<uint64_t>::max(),
      1).has_value());
}

TEST(ThrustEstimationInput, AcceptsOnlyFreshUnclippedPositiveAcceleration)
{
  const rclcpp::Time received_time(1'000'000'000LL);
  const auto fresh_time = received_time + rclcpp::Duration::from_seconds(0.1);
  const Eigen::Vector3d acceleration_flu(0.0, 0.0, 9.81);
  constexpr uint64_t sample_id = 1234;
  constexpr uint64_t previous_sample_id = 1233;

  EXPECT_TRUE(
    thrust_estimation_imu_sample_is_usable(
      true, false, acceleration_flu, sample_id, previous_sample_id,
      fresh_time, received_time, 0.5));
  EXPECT_FALSE(
    thrust_estimation_imu_sample_is_usable(
      false, false, acceleration_flu, sample_id, previous_sample_id,
      fresh_time, received_time, 0.5));
  EXPECT_FALSE(
    thrust_estimation_imu_sample_is_usable(
      true, true, acceleration_flu, sample_id, previous_sample_id,
      fresh_time, received_time, 0.5));
  EXPECT_FALSE(
    thrust_estimation_imu_sample_is_usable(
      true, false, Eigen::Vector3d::Zero(), sample_id, previous_sample_id,
      fresh_time, received_time, 0.5));
}

TEST(ThrustEstimationInput, AcceptsEachSampleOnlyOnce)
{
  const rclcpp::Time received_time(1'000'000'000LL);
  const Eigen::Vector3d acceleration_flu(0.0, 0.0, 9.81);

  EXPECT_TRUE(
    thrust_estimation_imu_sample_is_usable(
      true, false, acceleration_flu, 1234, 1233,
      received_time, received_time, 0.5));
  EXPECT_FALSE(
    thrust_estimation_imu_sample_is_usable(
      true, false, acceleration_flu, 1234, 1234,
      received_time, received_time, 0.5));
  EXPECT_TRUE(
    thrust_estimation_imu_sample_is_usable(
      true, false, acceleration_flu, 1235, 1234,
      received_time, received_time, 0.5));
}

TEST(ThrustEstimationInput, RejectsExpiredAndFutureDatedSamples)
{
  const rclcpp::Time received_time(1'000'000'000LL);
  const Eigen::Vector3d acceleration_flu(0.0, 0.0, 9.81);

  EXPECT_FALSE(
    thrust_estimation_imu_sample_is_usable(
      true,
      false,
      acceleration_flu,
      1234,
      1233,
      received_time + rclcpp::Duration::from_seconds(0.5),
      received_time,
      0.5));
  EXPECT_FALSE(
    thrust_estimation_imu_sample_is_usable(
      true,
      false,
      acceleration_flu,
      1234,
      1233,
      received_time - rclcpp::Duration::from_seconds(0.001),
      received_time,
      0.5));
}

}  // namespace px4_ctrl_ros2
