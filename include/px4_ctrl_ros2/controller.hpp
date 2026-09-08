#pragma once

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <queue>
#include <utility>

namespace px4_ctrl_ros2
{

struct ControlParams
{
  double mass{0.5};
  double gra{9.81};
  int pose_solver{1};
  double ctrl_freq_max{100.0};
  bool use_bodyrate_ctrl{false};
  double max_manual_vel{1.0};
  double max_angle_rad{M_PI / 6.0};
  double low_voltage{14.0};
  bool accurate_thrust_model{false};
  bool thrust_model_print_value{false};
  double thrust_model_k1{0.7583};
  double thrust_model_k2{1.6942};
  double thrust_model_k3{0.6786};
  double hover_percentage{0.40};
  Eigen::Vector3d rotor_drag{Eigen::Vector3d::Zero()};
  double rotor_drag_k_thrust_horz{0.0};
  Eigen::Vector3d kp{2.0, 2.0, 1.8};
  Eigen::Vector3d kv{2.0, 2.0, 2.0};
  Eigen::Vector3d kang{12.0, 12.0, 4.0};
  double pos_error_limit{1.5};
  double vel_error_limit{1.5};
};

struct OdomState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d w{Eigen::Vector3d::Zero()};
};

struct DesiredState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};
  Eigen::Vector3d j{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  double yaw_rate{0.0};
};

enum class ThrustEstimateStatus : uint8_t
{
  NOT_ATTEMPTED = 0,
  INVALID_INPUT,
  NO_THRUST_SAMPLE,
  WAITING_FOR_DELAY,
  STALE_THRUST_SAMPLE,
  INVALID_UPDATE,
  UPDATED
};

inline const char * thrust_estimate_status_name(ThrustEstimateStatus status)
{
  switch (status) {
    case ThrustEstimateStatus::NOT_ATTEMPTED: return "not_attempted";
    case ThrustEstimateStatus::INVALID_INPUT: return "invalid_input";
    case ThrustEstimateStatus::NO_THRUST_SAMPLE: return "no_thrust_sample";
    case ThrustEstimateStatus::WAITING_FOR_DELAY: return "waiting_for_delay";
    case ThrustEstimateStatus::STALE_THRUST_SAMPLE: return "stale_thrust_sample";
    case ThrustEstimateStatus::INVALID_UPDATE: return "invalid_update";
    case ThrustEstimateStatus::UPDATED: return "updated";
  }
  return "unknown";
}

struct ControllerOutput
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d bodyrates{Eigen::Vector3d::Zero()};
  // Finite controller outputs are normalized to the PX4 command range [0, 1].
  double thrust{0.0};
};

struct DebugValues
{
  Eigen::Vector3d pid_acc{Eigen::Vector3d::Zero()};
  Eigen::Vector3d total_acc{Eigen::Vector3d::Zero()};
  Eigen::Vector3d desired_velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond desired_q{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d feedback_bodyrates{Eigen::Vector3d::Zero()};
  double desired_acc_norm{0.0};
  double normalized_thrust{0.0};
  double thr2acc{0.0};
  double hover_percentage{0.0};
  bool thrust_estimate_clamped{false};
  ThrustEstimateStatus thrust_estimate_status{ThrustEstimateStatus::NOT_ATTEMPTED};
  double thrust_estimate_body_acc_z{NAN};
  double thrust_estimate_elapsed{NAN};
  double thrust_estimate_input_thrust{NAN};
  std::size_t thrust_estimate_queue_size{0};
};

class Controller
{
public:
  explicit Controller(ControlParams params);

  void set_params(const ControlParams &params);
  const ControlParams &params() const { return params_; }
  const DebugValues &debug() const { return debug_; }

  ControllerOutput update_alg1(
    const DesiredState &des,
    const OdomState &odom,
    double voltage,
    const rclcpp::Time &command_time);
  bool estimate_thrust_model(double body_acc_z, const rclcpp::Time &sample_time);
  double thr2acc() const { return thr2acc_; }
  void reset_thrust_mapping();

private:
  static constexpr double kMinNormalizedCollectiveThrust = 3.0;
  static constexpr double kAlmostZeroValueThreshold = 0.001;

  ControlParams params_{};
  DebugValues debug_{};
  double thr2acc_{9.81 / 0.40};
  std::queue<std::pair<rclcpp::Time, double>> timed_thrust_;
  static constexpr double kThrustModelRho2 = 0.998;
  static constexpr double kMinEstimatedHoverPercentage = 0.1;
  static constexpr double kMaxEstimatedHoverPercentage = 0.8;
  static constexpr double kThrustDelayMinSeconds = 0.025; // gazebo 使用0.020
  static constexpr double kThrustDelayMaxSeconds = 0.040; // gazebo 使用0.060
  static constexpr std::size_t kMaxTimedThrustSamples = 100;
  double P_{1.0e6};

  Eigen::Vector3d compute_pid_error_acc(const OdomState &odom, const DesiredState &des);
  Eigen::Vector3d compute_limited_total_acc(
    const Eigen::Vector3d &pid_error_acc,
    const Eigen::Vector3d &ref_acc) const;
  double compute_desired_collective_thrust(
    const Eigen::Quaterniond &est_q,
    const Eigen::Vector3d &est_v,
    const Eigen::Vector3d &des_acc,
    double voltage);
  double accurate_thrust_acc_mapping(double des_acc_z, double voltage) const;
  Eigen::Quaterniond compute_flat_attitude(
    const Eigen::Vector3d &thr_acc,
    const Eigen::Vector3d &jerk,
    double yaw,
    double yaw_rate,
    const Eigen::Quaterniond &att_est,
    Eigen::Vector3d &bodyrates) const;
  Eigen::Vector3d compute_feedback_bodyrates(
    const Eigen::Quaterniond &des_q,
    const Eigen::Quaterniond &est_q) const;
};

double normalize_angle(double yaw);
Eigen::Quaterniond ned_frd_to_nwu_flu(const Eigen::Quaterniond &q_ned_frd);
Eigen::Quaterniond nwu_flu_to_ned_frd(const Eigen::Quaterniond &q_nwu_flu);

}  // namespace px4_ctrl_ros2
