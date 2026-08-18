#pragma once

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
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

inline std::optional<uint64_t> sensor_acceleration_timestamp_us(
  uint64_t message_timestamp_us,
  int32_t accelerometer_timestamp_relative_us)
{
  if (message_timestamp_us == 0 ||
    accelerometer_timestamp_relative_us == std::numeric_limits<int32_t>::max())
  {
    return std::nullopt;
  }

  if (accelerometer_timestamp_relative_us >= 0) {
    const auto offset_us = static_cast<uint64_t>(accelerometer_timestamp_relative_us);
    if (message_timestamp_us > std::numeric_limits<uint64_t>::max() - offset_us) {
      return std::nullopt;
    }
    return message_timestamp_us + offset_us;
  }

  const auto offset_us = static_cast<uint64_t>(
    -static_cast<int64_t>(accelerometer_timestamp_relative_us));
  if (message_timestamp_us <= offset_us) {
    return std::nullopt;
  }
  return message_timestamp_us - offset_us;
}

inline bool thrust_estimation_imu_sample_is_usable(
  bool have_sample,
  bool accelerometer_clipped,
  const Eigen::Vector3d & acceleration_flu,
  uint64_t sample_id,
  uint64_t last_consumed_sample_id,
  const rclcpp::Time & sample_time,
  const rclcpp::Time & received_time,
  double timeout_seconds)
{
  if (!have_sample || sample_id == 0 || sample_id == last_consumed_sample_id ||
    accelerometer_clipped ||
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
