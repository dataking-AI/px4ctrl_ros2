#include "flight_state.hpp"
#include "land_detector.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace px4_ctrl_ros2
{

namespace
{

rclcpp::Time time_at(double seconds)
{
  return rclcpp::Time(0, 0, RCL_ROS_TIME) + rclcpp::Duration::from_seconds(seconds);
}

// The drone hovers at 5 m while the controller keeps asking for 4 m: low, but
// not low enough for constraint 1.
constexpr double kHoverZ = 5.0;
constexpr double kShallowDesiredZ = 4.8;
// More than 0.5 m below the estimate, so constraint 1 holds.
constexpr double kDeepDesiredZ = 4.0;
constexpr double kStillSpeed = 0.05;
constexpr double kMovingSpeed = 0.5;

}  // namespace

TEST(LandDetector, StartsLandedAndStaysLandedUntilPilotHandsOver)
{
  LandDetector detector;
  EXPECT_TRUE(detector.landed);

  detector.update(
    FlightState::AUTO_HOVER, FlightState::AUTO_HOVER, true,
    kHoverZ, kHoverZ, kStillSpeed, time_at(0.0));
  EXPECT_TRUE(detector.landed);

  detector.update(
    FlightState::MANUAL_CTRL, FlightState::AUTO_HOVER, true,
    kHoverZ, kHoverZ, kStillSpeed, time_at(0.1));
  EXPECT_FALSE(detector.landed);
}

TEST(LandDetector, ManualControlWithoutArmingMeansOnTheGround)
{
  LandDetector detector;
  detector.update(
    FlightState::MANUAL_CTRL, FlightState::AUTO_HOVER, true,
    kHoverZ, kHoverZ, kStillSpeed, time_at(0.0));
  ASSERT_FALSE(detector.landed);

  detector.update(
    FlightState::MANUAL_CTRL, FlightState::MANUAL_CTRL, false,
    kHoverZ, kHoverZ, kStillSpeed, time_at(0.5));
  EXPECT_TRUE(detector.landed);
}

TEST(LandDetector, ManualControlWhileArmedDoesNotForceGroundContact)
{
  LandDetector detector;
  detector.update(
    FlightState::MANUAL_CTRL, FlightState::AUTO_HOVER, true,
    kHoverZ, kHoverZ, kStillSpeed, time_at(0.0));
  ASSERT_FALSE(detector.landed);

  detector.update(
    FlightState::MANUAL_CTRL, FlightState::MANUAL_CTRL, true,
    kHoverZ, kHoverZ, kStillSpeed, time_at(0.5));
  EXPECT_FALSE(detector.landed);
}

TEST(LandDetector, IgnoresASetpointThatIsNotFarEnoughBelowTheEstimate)
{
  LandDetector detector;
  detector.update(
    FlightState::MANUAL_CTRL, FlightState::AUTO_TAKEOFF, true,
    kHoverZ, kHoverZ, kStillSpeed, time_at(0.0));
  ASSERT_FALSE(detector.landed);

  for (double t = 1.0; t < 10.0; t += 1.0) {
    detector.update(
      FlightState::AUTO_LAND, FlightState::AUTO_LAND, true,
      kShallowDesiredZ, kHoverZ, kStillSpeed, time_at(t));
  }
  EXPECT_FALSE(detector.landed);
  EXPECT_FALSE(detector.condition_started());
}

TEST(LandDetector, NeedsThreeSecondsOfStillnessBeforeTouchdown)
{
  LandDetector detector;
  detector.update(
    FlightState::MANUAL_CTRL, FlightState::AUTO_TAKEOFF, true,
    kHoverZ, kHoverZ, kStillSpeed, time_at(0.0));
  ASSERT_FALSE(detector.landed);

  detector.update(
    FlightState::AUTO_LAND, FlightState::AUTO_LAND, true,
    kDeepDesiredZ, kHoverZ, kStillSpeed, time_at(2.0));
  EXPECT_FALSE(detector.landed);
  EXPECT_TRUE(detector.condition_started());

  detector.update(
    FlightState::AUTO_LAND, FlightState::AUTO_LAND, true,
    kDeepDesiredZ, kHoverZ, kStillSpeed, time_at(4.9));
  EXPECT_FALSE(detector.landed);

  detector.update(
    FlightState::AUTO_LAND, FlightState::AUTO_LAND, true,
    kDeepDesiredZ, kHoverZ, kStillSpeed, time_at(5.1));
  EXPECT_TRUE(detector.landed);
}

TEST(LandDetector, RestartsTheTimerWhenTheConditionBreaks)
{
  LandDetector detector;
  detector.update(
    FlightState::MANUAL_CTRL, FlightState::AUTO_TAKEOFF, true,
    kHoverZ, kHoverZ, kStillSpeed, time_at(0.0));

  detector.update(
    FlightState::AUTO_LAND, FlightState::AUTO_LAND, true,
    kDeepDesiredZ, kHoverZ, kStillSpeed, time_at(1.0));
  ASSERT_TRUE(detector.condition_started());

  // The drone starts moving again: the elapsed time is discarded.
  detector.update(
    FlightState::AUTO_LAND, FlightState::AUTO_LAND, true,
    kDeepDesiredZ, kHoverZ, kMovingSpeed, time_at(2.5));
  EXPECT_FALSE(detector.condition_started());
  EXPECT_FALSE(detector.landed);

  detector.update(
    FlightState::AUTO_LAND, FlightState::AUTO_LAND, true,
    kDeepDesiredZ, kHoverZ, kStillSpeed, time_at(3.0));
  EXPECT_TRUE(detector.condition_started());

  detector.update(
    FlightState::AUTO_LAND, FlightState::AUTO_LAND, true,
    kDeepDesiredZ, kHoverZ, kStillSpeed, time_at(5.5));
  EXPECT_FALSE(detector.landed);

  detector.update(
    FlightState::AUTO_LAND, FlightState::AUTO_LAND, true,
    kDeepDesiredZ, kHoverZ, kStillSpeed, time_at(6.5));
  EXPECT_TRUE(detector.landed);
}

TEST(LandDetector, OnceLandedOnlyAPilotHandoverClearsIt)
{
  LandDetector detector;
  detector.update(
    FlightState::MANUAL_CTRL, FlightState::AUTO_TAKEOFF, true,
    kHoverZ, kHoverZ, kStillSpeed, time_at(0.0));
  detector.update(
    FlightState::AUTO_LAND, FlightState::AUTO_LAND, true,
    kDeepDesiredZ, kHoverZ, kStillSpeed, time_at(1.0));
  detector.update(
    FlightState::AUTO_LAND, FlightState::AUTO_LAND, true,
    kDeepDesiredZ, kHoverZ, kStillSpeed, time_at(5.0));
  ASSERT_TRUE(detector.landed);

  // Climbing again while still in automatic control keeps the flag set.
  detector.update(
    FlightState::AUTO_LAND, FlightState::AUTO_HOVER, true,
    kHoverZ, kHoverZ, kMovingSpeed, time_at(6.0));
  EXPECT_TRUE(detector.landed);

  detector.update(
    FlightState::MANUAL_CTRL, FlightState::AUTO_HOVER, true,
    kHoverZ, kHoverZ, kMovingSpeed, time_at(7.0));
  EXPECT_FALSE(detector.landed);
}

TEST(LandDetector, IgnoresNonFiniteInputs)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  LandDetector detector;
  detector.update(
    FlightState::MANUAL_CTRL, FlightState::AUTO_TAKEOFF, true,
    kHoverZ, kHoverZ, kStillSpeed, time_at(0.0));

  detector.update(
    FlightState::AUTO_LAND, FlightState::AUTO_LAND, true,
    nan, kHoverZ, kStillSpeed, time_at(1.0));
  EXPECT_FALSE(detector.condition_started());

  detector.update(
    FlightState::AUTO_LAND, FlightState::AUTO_LAND, true,
    kDeepDesiredZ, nan, kStillSpeed, time_at(2.0));
  EXPECT_FALSE(detector.condition_started());

  detector.update(
    FlightState::AUTO_LAND, FlightState::AUTO_LAND, true,
    kDeepDesiredZ, kHoverZ, nan, time_at(3.0));
  EXPECT_FALSE(detector.condition_started());
  EXPECT_FALSE(detector.landed);
}

}  // namespace px4_ctrl_ros2
