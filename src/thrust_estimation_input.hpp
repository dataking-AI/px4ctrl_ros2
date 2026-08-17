#pragma once

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <cmath>
#include <optional>

namespace px4_ctrl_ros2
{

inline std::optional<Eigen::Vector3d> sensor_acceleration_frd_to_flu(
  const std::array<float, 3> & acceleration_frd)
{
  const Eigen::Vector3d acceleration_flu(
    acceleration_frd[0], -acceleration_frd[1], -acceleration_frd[2]);
  if (!std::isfinite(acceleration_flu.x()) ||
    !std::isfinite(acceleration_flu.y()) ||
    !std::isfinite(acceleration_flu.z()))
  {
    return std::nullopt;
  }
  return acceleration_flu;
}

inline bool thrust_estimation_imu_sample_is_usable(
  bool have_sample,
  bool accelerometer_clipped,
  const Eigen::Vector3d & acceleration_flu,
  const rclcpp::Time & sample_time,
  const rclcpp::Time & received_time,
  double timeout_seconds)
{
  if (!have_sample || accelerometer_clipped ||
    !std::isfinite(acceleration_flu.x()) ||
    !std::isfinite(acceleration_flu.y()) ||
    !std::isfinite(acceleration_flu.z()) ||
    acceleration_flu.z() <= 0.0)
  {
    return false;
  }

  const double age_seconds = (sample_time - received_time).seconds();
  return age_seconds >= 0.0 && age_seconds < timeout_seconds;
}

}  // namespace px4_ctrl_ros2
