#include "px4_ctrl_ros2/controller.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace px4_ctrl_ros2
{

namespace
{
constexpr double kVoltage = 14.0;

Controller make_controller()
{
  ControlParams params{};
  params.gra = 9.81;
  params.hover_percentage = 0.4;
  return Controller(params);
}

double enqueue_hover_thrust(Controller &controller, const rclcpp::Time &time)
{
  const DesiredState desired{};
  const OdomState odom{};
  return controller.update_alg1(desired, odom, kVoltage, time).thrust;
}
}  // namespace

TEST(ThrustMapping, WaitsForTheMinimumDelay)
{
  auto controller = make_controller();
  const rclcpp::Time command_time(1'000'000'000LL);
  const double initial_thr2acc = controller.thr2acc();
  enqueue_hover_thrust(controller, command_time);

  EXPECT_FALSE(controller.estimate_thrust_model(12.0, command_time + rclcpp::Duration::from_seconds(0.020)));
  EXPECT_DOUBLE_EQ(initial_thr2acc, controller.thr2acc());
}

TEST(ThrustMapping, UpdatesWithACommandInsideTheDelayWindow)
{
  auto controller = make_controller();
  const rclcpp::Time command_time(1'000'000'000LL);
  enqueue_hover_thrust(controller, command_time);

  EXPECT_TRUE(controller.estimate_thrust_model(12.0, command_time + rclcpp::Duration::from_seconds(0.040)));
  EXPECT_NEAR(30.0, controller.thr2acc(), 1.0e-3);
  EXPECT_NEAR(9.81 / 30.0, controller.debug().hover_percentage, 1.0e-5);
  EXPECT_FALSE(controller.debug().thrust_estimate_clamped);
}

TEST(ThrustMapping, PrintOptionDoesNotChangeEstimation)
{
  ControlParams quiet_params{};
  quiet_params.gra = 9.81;
  quiet_params.hover_percentage = 0.4;
  ControlParams print_params = quiet_params;
  print_params.thrust_model_print_value = true;
  Controller quiet_controller(quiet_params);
  Controller print_controller(print_params);
  const rclcpp::Time command_time(1'000'000'000LL);

  enqueue_hover_thrust(quiet_controller, command_time);
  enqueue_hover_thrust(print_controller, command_time);
  const auto sample_time = command_time + rclcpp::Duration::from_seconds(0.040);

  EXPECT_TRUE(quiet_controller.estimate_thrust_model(12.0, sample_time));
  EXPECT_TRUE(print_controller.estimate_thrust_model(12.0, sample_time));
  EXPECT_DOUBLE_EQ(quiet_controller.thr2acc(), print_controller.thr2acc());
  EXPECT_DOUBLE_EQ(
    quiet_controller.debug().hover_percentage,
    print_controller.debug().hover_percentage);
}

TEST(ThrustMapping, ClampsEstimatedHoverPercentageAtLowerBound)
{
  auto controller = make_controller();
  const rclcpp::Time command_time(1'000'000'000LL);
  enqueue_hover_thrust(controller, command_time);

  EXPECT_TRUE(controller.estimate_thrust_model(
      100.0, command_time + rclcpp::Duration::from_seconds(0.040)));
  EXPECT_DOUBLE_EQ(0.1, controller.debug().hover_percentage);
  EXPECT_DOUBLE_EQ(9.81 / 0.1, controller.thr2acc());
  EXPECT_TRUE(controller.debug().thrust_estimate_clamped);
}

TEST(ThrustMapping, ClampsEstimatedHoverPercentageAtUpperBound)
{
  auto controller = make_controller();
  const rclcpp::Time command_time(1'000'000'000LL);
  enqueue_hover_thrust(controller, command_time);

  EXPECT_TRUE(controller.estimate_thrust_model(
      1.0, command_time + rclcpp::Duration::from_seconds(0.040)));
  EXPECT_DOUBLE_EQ(0.8, controller.debug().hover_percentage);
  EXPECT_DOUBLE_EQ(9.81 / 0.8, controller.thr2acc());
  EXPECT_TRUE(controller.debug().thrust_estimate_clamped);
}

TEST(ThrustMapping, ConvergesToTheMeasuredThrustAccelerationRatio)
{
  auto controller = make_controller();
  const double expected_thr2acc = 28.0;
  const rclcpp::Time start_time(1'000'000'000LL);

  for (int i = 0; i < 5; ++i) {
    const auto command_time = start_time + rclcpp::Duration::from_seconds(0.1 * i);
    const double thrust = enqueue_hover_thrust(controller, command_time);
    EXPECT_TRUE(controller.estimate_thrust_model(
        expected_thr2acc * thrust,
        command_time + rclcpp::Duration::from_seconds(0.040)));
  }

  EXPECT_NEAR(expected_thr2acc, controller.thr2acc(), 1.0e-2);
}

TEST(ThrustMapping, DiscardsCommandsOlderThanTheDelayWindow)
{
  auto controller = make_controller();
  const rclcpp::Time command_time(1'000'000'000LL);
  const double initial_thr2acc = controller.thr2acc();
  enqueue_hover_thrust(controller, command_time);

  EXPECT_FALSE(controller.estimate_thrust_model(12.0, command_time + rclcpp::Duration::from_seconds(0.046)));
  EXPECT_DOUBLE_EQ(initial_thr2acc, controller.thr2acc());
}

TEST(ThrustMapping, SaturatesHighCommandAndIdentifiesUsingThePublishedThrust)
{
  auto controller = make_controller();
  const rclcpp::Time command_time(1'000'000'000LL);
  DesiredState desired{};
  desired.a.z() = 30.0;
  const OdomState odom{};

  const auto output = controller.update_alg1(desired, odom, kVoltage, command_time);

  EXPECT_DOUBLE_EQ(1.0, output.thrust);
  EXPECT_DOUBLE_EQ(1.0, controller.debug().normalized_thrust);
  EXPECT_TRUE(controller.estimate_thrust_model(
      30.0, command_time + rclcpp::Duration::from_seconds(0.040)));
  EXPECT_NEAR(30.0, controller.thr2acc(), 1.0e-3);
}

TEST(ThrustMapping, SaturatesLowCommandAndDoesNotQueueZeroThrust)
{
  ControlParams params{};
  params.gra = 9.81;
  params.hover_percentage = 0.4;
  params.rotor_drag_k_thrust_horz = 1.0;
  Controller controller(params);
  const rclcpp::Time command_time(1'000'000'000LL);
  const DesiredState desired{};
  OdomState odom{};
  odom.v.x() = 10.0;

  const auto output = controller.update_alg1(desired, odom, kVoltage, command_time);

  EXPECT_DOUBLE_EQ(0.0, output.thrust);
  EXPECT_DOUBLE_EQ(0.0, controller.debug().normalized_thrust);
  EXPECT_FALSE(controller.estimate_thrust_model(
      12.0, command_time + rclcpp::Duration::from_seconds(0.040)));
  EXPECT_DOUBLE_EQ(9.81 / 0.4, controller.thr2acc());
}

TEST(ThrustMapping, ResetClearsQueuedCommandsAndRestoresFastConvergence)
{
  auto controller = make_controller();
  const rclcpp::Time command_time(1'000'000'000LL);
  const double initial_thr2acc = controller.thr2acc();
  enqueue_hover_thrust(controller, command_time);
  EXPECT_TRUE(controller.estimate_thrust_model(
      12.0, command_time + rclcpp::Duration::from_seconds(0.040)));
  EXPECT_NE(initial_thr2acc, controller.thr2acc());
  controller.reset_thrust_mapping();

  EXPECT_FALSE(controller.estimate_thrust_model(12.0, command_time + rclcpp::Duration::from_seconds(0.040)));
  EXPECT_DOUBLE_EQ(initial_thr2acc, controller.thr2acc());
  EXPECT_DOUBLE_EQ(0.4, controller.debug().hover_percentage);
  EXPECT_FALSE(controller.debug().thrust_estimate_clamped);

  const auto next_command_time = command_time + rclcpp::Duration::from_seconds(0.100);
  enqueue_hover_thrust(controller, next_command_time);
  EXPECT_TRUE(controller.estimate_thrust_model(12.0, next_command_time + rclcpp::Duration::from_seconds(0.040)));
  EXPECT_NEAR(30.0, controller.thr2acc(), 1.0e-3);
}

TEST(ThrustMapping, RejectsInvalidObservationsAndNonPositiveThrust)
{
  auto controller = make_controller();
  const rclcpp::Time command_time(1'000'000'000LL);
  const double initial_thr2acc = controller.thr2acc();
  enqueue_hover_thrust(controller, command_time);
  EXPECT_FALSE(controller.estimate_thrust_model(NAN, command_time + rclcpp::Duration::from_seconds(0.040)));
  EXPECT_DOUBLE_EQ(initial_thr2acc, controller.thr2acc());

  EXPECT_FALSE(controller.estimate_thrust_model(0.0, command_time + rclcpp::Duration::from_seconds(0.040)));
  EXPECT_DOUBLE_EQ(initial_thr2acc, controller.thr2acc());

  ControlParams invalid_mapping_params{};
  invalid_mapping_params.accurate_thrust_model = true;
  invalid_mapping_params.thrust_model_k1 = -1.0;
  invalid_mapping_params.thrust_model_k3 = 1.0;
  Controller invalid_mapping_controller(invalid_mapping_params);
  enqueue_hover_thrust(invalid_mapping_controller, command_time);
  const double invalid_initial = invalid_mapping_controller.thr2acc();
  EXPECT_FALSE(invalid_mapping_controller.estimate_thrust_model(
      12.0, command_time + rclcpp::Duration::from_seconds(0.040)));
  EXPECT_DOUBLE_EQ(invalid_initial, invalid_mapping_controller.thr2acc());
}

}  // namespace px4_ctrl_ros2
