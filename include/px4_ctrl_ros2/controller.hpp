#pragma once

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <cstdint>

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

struct ControllerOutput
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d bodyrates{Eigen::Vector3d::Zero()};
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
};

class Controller
{
public:
  explicit Controller(ControlParams params);

  void set_params(const ControlParams &params);
  const ControlParams &params() const { return params_; }
  const DebugValues &debug() const { return debug_; }

  ControllerOutput update_alg1(const DesiredState &des, const OdomState &odom, double voltage);
  void reset_thrust_mapping();

private:
  static constexpr double kMinNormalizedCollectiveThrust = 3.0;
  static constexpr double kAlmostZeroValueThreshold = 0.001;

  ControlParams params_{};
  DebugValues debug_{};
  double thr2acc_{9.81 / 0.40};

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
Eigen::Vector3d enu_to_ned(const Eigen::Vector3d &enu);
Eigen::Vector3d ned_to_enu(const Eigen::Vector3d &ned);
Eigen::Quaterniond ned_frd_to_enu_flu(const Eigen::Quaterniond &q_ned_frd);
Eigen::Quaterniond enu_flu_to_ned_frd(const Eigen::Quaterniond &q_enu_flu);

}  // namespace px4_ctrl_ros2
