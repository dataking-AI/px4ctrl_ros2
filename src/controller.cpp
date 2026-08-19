#include "px4_ctrl_ros2/controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace px4_ctrl_ros2
{

namespace
{
Eigen::Matrix3d enu_to_ned_matrix()
{
  Eigen::Matrix3d m;
  m << 0.0, 1.0, 0.0,
       1.0, 0.0, 0.0,
       0.0, 0.0, -1.0;
  return m;
}

Eigen::Matrix3d flu_to_frd_matrix()
{
  Eigen::Matrix3d m;
  m << 1.0, 0.0, 0.0,
       0.0, -1.0, 0.0,
       0.0, 0.0, -1.0;
  return m;
}

}  // namespace

Controller::Controller(ControlParams params) : params_(std::move(params))
{
  reset_thrust_mapping();
}

void Controller::set_params(const ControlParams &params)
{
  params_ = params;
  reset_thrust_mapping();
}

void Controller::reset_thrust_mapping()
{
  const double hover_percentage = std::clamp(params_.hover_percentage, 0.05, 0.95);
  thr2acc_ = params_.gra / hover_percentage;
  P_ = 1.0e6;
  timed_thrust_ = {};
  debug_.thr2acc = thr2acc_;
  debug_.hover_percentage = hover_percentage;
  debug_.thrust_estimate_clamped = false;
}

ControllerOutput Controller::update_alg1(
  const DesiredState &des,
  const OdomState &odom,
  double voltage,
  const rclcpp::Time &command_time)
{
  ControllerOutput output{};
  const Eigen::Vector3d pid_error_acc = compute_pid_error_acc(odom, des);
  const Eigen::Vector3d total_des_acc = compute_limited_total_acc(pid_error_acc, des.a);
  const double requested_thrust =
    compute_desired_collective_thrust(odom.q, odom.v, total_des_acc, voltage);
  output.thrust = std::isfinite(requested_thrust) ?
    std::clamp(requested_thrust, 0.0, 1.0) : requested_thrust;
  output.q = compute_flat_attitude(total_des_acc, des.j, des.yaw, des.yaw_rate, odom.q, output.bodyrates);
  const Eigen::Vector3d feedback_bodyrates = compute_feedback_bodyrates(output.q, odom.q);
  output.bodyrates += feedback_bodyrates;

  debug_.pid_acc = pid_error_acc;
  debug_.total_acc = total_des_acc;
  debug_.desired_q = output.q;
  debug_.feedback_bodyrates = feedback_bodyrates;
  debug_.normalized_thrust = output.thrust;
  debug_.thr2acc = thr2acc_;

  if (std::isfinite(output.thrust) && output.thrust > 0.0) {
    timed_thrust_.emplace(command_time, output.thrust);
    while (timed_thrust_.size() > kMaxTimedThrustSamples) {
      timed_thrust_.pop();
    }
  }
  return output;
}

bool Controller::estimate_thrust_model(double body_acc_z, const rclcpp::Time &sample_time)
{
  debug_.thrust_estimate_clamped = false;
  if (!std::isfinite(body_acc_z) || body_acc_z <= 0.0 ||
    !std::isfinite(params_.gra) || params_.gra <= 0.0)
  {
    return false;
  }

  while (!timed_thrust_.empty()) {
    const auto &timed_thrust = timed_thrust_.front();
    const double elapsed = (sample_time - timed_thrust.first).seconds();
    if (elapsed > kThrustDelayMaxSeconds) {
      timed_thrust_.pop();
      continue;
    }
    if (elapsed < kThrustDelayMinSeconds) {
      return false;
    }

    const double thrust = timed_thrust.second;
    timed_thrust_.pop();
    if (!std::isfinite(thrust) || thrust <= 0.0) {
      return false;
    }

    const double denominator = kThrustModelRho2 + thrust * P_ * thrust;
    if (!std::isfinite(denominator) || denominator <= 0.0) {
      return false;
    }

    const double gain = P_ * thrust / denominator;
    const double updated_thr2acc = thr2acc_ + gain * (body_acc_z - thrust * thr2acc_);
    const double updated_covariance = (1.0 - gain * thrust) * P_ / kThrustModelRho2;
    if (!std::isfinite(updated_thr2acc) || updated_thr2acc <= 0.0 ||
      !std::isfinite(updated_covariance) || updated_covariance <= 0.0)
    {
      return false;
    }

    const double estimated_hover_percentage = params_.gra / updated_thr2acc;
    if (!std::isfinite(estimated_hover_percentage) || estimated_hover_percentage <= 0.0) {
      return false;
    }
    const double bounded_hover_percentage = std::clamp(
      estimated_hover_percentage,
      kMinEstimatedHoverPercentage,
      kMaxEstimatedHoverPercentage);

    thr2acc_ = params_.gra / bounded_hover_percentage;
    P_ = updated_covariance;
    debug_.thr2acc = thr2acc_;
    debug_.hover_percentage = bounded_hover_percentage;
    debug_.thrust_estimate_clamped = bounded_hover_percentage != estimated_hover_percentage;
    return true;
  }
  return false;
}

Eigen::Vector3d Controller::compute_pid_error_acc(const OdomState &odom, const DesiredState &des)
{
  Eigen::Vector3d acc_error;
  Eigen::Vector3d desired_velocity;
  const double pos_error_limit = std::max(0.0, params_.pos_error_limit);
  const double vel_error_limit = std::max(0.0, params_.vel_error_limit);
  for (int i = 0; i < 3; ++i) {
    const double pos_error = std::isnan(des.p(i)) ?
      0.0 : std::clamp(des.p(i) - odom.p(i), -pos_error_limit, pos_error_limit);
    desired_velocity(i) = des.v(i) + params_.kp(i) * pos_error;
    const double vel_error =
      std::clamp(desired_velocity(i) - odom.v(i), -vel_error_limit, vel_error_limit);
    acc_error(i) = params_.kv(i) * vel_error;
  }
  debug_.desired_velocity = desired_velocity;
  return acc_error;
}

Eigen::Vector3d Controller::compute_limited_total_acc(
  const Eigen::Vector3d &pid_error_acc,
  const Eigen::Vector3d &ref_acc) const
{
  Eigen::Vector3d total_acc = pid_error_acc + ref_acc + Eigen::Vector3d(0.0, 0.0, params_.gra);

  if (params_.max_angle_rad > 0.0 && total_acc.norm() > kAlmostZeroValueThreshold) {
    double z_acc = total_acc.dot(Eigen::Vector3d::UnitZ());
    Eigen::Vector3d z_b = total_acc.normalized();
    if (z_acc < kMinNormalizedCollectiveThrust) {
      z_acc = kMinNormalizedCollectiveThrust;
    }

    const double dot = std::clamp(Eigen::Vector3d::UnitZ().dot(z_b), -1.0, 1.0);
    const double rot_ang = std::acos(dot);
    if (rot_ang > params_.max_angle_rad) {
      Eigen::Vector3d rot_axis = Eigen::Vector3d::UnitZ().cross(z_b);
      if (rot_axis.norm() > kAlmostZeroValueThreshold) {
        rot_axis.normalize();
        const Eigen::Vector3d limited_z_b =
          Eigen::AngleAxisd(params_.max_angle_rad, rot_axis) * Eigen::Vector3d::UnitZ();
        total_acc = z_acc / std::cos(params_.max_angle_rad) * limited_z_b;
      }
    }
  }

  return total_acc;
}

double Controller::compute_desired_collective_thrust(
  const Eigen::Quaterniond &est_q,
  const Eigen::Vector3d &est_v,
  const Eigen::Vector3d &des_acc,
  double voltage)
{
  const Eigen::Vector3d body_z_axis = est_q * Eigen::Vector3d::UnitZ();
  double des_acc_norm = des_acc.dot(body_z_axis);
  if (des_acc_norm < kMinNormalizedCollectiveThrust) {
    des_acc_norm = kMinNormalizedCollectiveThrust;
  }
  des_acc_norm -= params_.rotor_drag_k_thrust_horz *
    (std::pow(est_v.x(), 2.0) + std::pow(est_v.y(), 2.0));

  debug_.desired_acc_norm = des_acc_norm;

  if (params_.accurate_thrust_model) {
    return accurate_thrust_acc_mapping(des_acc_norm, voltage);
  }
  return des_acc_norm / thr2acc_;
}

double Controller::accurate_thrust_acc_mapping(double des_acc_z, double voltage) const
{
  voltage = std::clamp(voltage, params_.low_voltage, 1.5 * params_.low_voltage);
  const double a = params_.thrust_model_k3;
  const double b = 1.0 - params_.thrust_model_k3;
  const double c = -(params_.mass * des_acc_z) /
    (params_.thrust_model_k1 * std::pow(voltage, params_.thrust_model_k2));
  const double discriminant = std::max(0.0, b * b - 4.0 * a * c);
  if (std::abs(a) < kAlmostZeroValueThreshold) {
    return std::abs(b) < kAlmostZeroValueThreshold ? 0.0 : -c / b;
  }
  return (-b + std::sqrt(discriminant)) / (2.0 * a);
}

Eigen::Quaterniond Controller::compute_flat_attitude(
  const Eigen::Vector3d &thr_acc,
  const Eigen::Vector3d &jerk,
  double yaw,
  double yaw_rate,
  const Eigen::Quaterniond &att_est,
  Eigen::Vector3d &bodyrates) const
{
  bodyrates.setZero();
  if (thr_acc.norm() < kMinNormalizedCollectiveThrust) {
    return att_est;
  }

  const Eigen::Vector3d zb = thr_acc.normalized();
  const double syaw = std::sin(yaw);
  const double cyaw = std::cos(yaw);
  const Eigen::Vector3d xc(cyaw, syaw, 0.0);
  const Eigen::Vector3d xcd(-syaw * yaw_rate, cyaw * yaw_rate, 0.0);
  Eigen::Vector3d yc = zb.cross(xc);
  if (yc.norm() < kAlmostZeroValueThreshold) {
    return att_est;
  }
  yc.normalize();
  const Eigen::Vector3d xb = yc.cross(zb);

  if (jerk.norm() > kAlmostZeroValueThreshold) {
    const double x_sqr_norm = thr_acc.squaredNorm();
    const double x_norm = std::sqrt(x_sqr_norm);
    const Eigen::Vector3d zbd = (jerk - thr_acc * (thr_acc.dot(jerk) / x_sqr_norm)) / x_norm;
    const Eigen::Vector3d ycd = zbd.cross(xc) + zb.cross(xcd);
    const Eigen::Vector3d ybd = (ycd - yc * yc.dot(ycd)) / std::max(yc.norm(), kAlmostZeroValueThreshold);
    const Eigen::Vector3d xbd = ybd.cross(zb) + yc.cross(zbd);
    bodyrates.x() = (zb.dot(ybd) - yc.dot(zbd)) / 2.0;
    bodyrates.y() = (xb.dot(zbd) - zb.dot(xbd)) / 2.0;
    bodyrates.z() = (yc.dot(xbd) - xb.dot(ybd)) / 2.0;
  }

  Eigen::Matrix3d rot_m;
  rot_m.col(0) = xb;
  rot_m.col(1) = yc;
  rot_m.col(2) = zb;
  return Eigen::Quaterniond(rot_m).normalized();
}

Eigen::Vector3d Controller::compute_feedback_bodyrates(
  const Eigen::Quaterniond &des_q,
  const Eigen::Quaterniond &est_q) const
{
  const Eigen::Quaterniond q_e = est_q.inverse() * des_q;
  const double sign = q_e.w() >= 0.0 ? 1.0 : -1.0;
  return Eigen::Vector3d(
    sign * 2.0 * params_.kang.x() * q_e.x(),
    sign * 2.0 * params_.kang.y() * q_e.y(),
    sign * 2.0 * params_.kang.z() * q_e.z());
}

double normalize_angle(double yaw)
{
  while (yaw > M_PI) {
    yaw -= 2.0 * M_PI;
  }
  while (yaw < -M_PI) {
    yaw += 2.0 * M_PI;
  }
  return yaw;
}

Eigen::Quaterniond ned_frd_to_enu_flu(const Eigen::Quaterniond &q_ned_frd)
{
  const Eigen::Matrix3d r_enu_flu =
    enu_to_ned_matrix().transpose() * q_ned_frd.toRotationMatrix() * flu_to_frd_matrix();
  return Eigen::Quaterniond(r_enu_flu).normalized();
}

Eigen::Quaterniond enu_flu_to_ned_frd(const Eigen::Quaterniond &q_enu_flu)
{
  const Eigen::Matrix3d r_ned_frd =
    enu_to_ned_matrix() * q_enu_flu.toRotationMatrix() * flu_to_frd_matrix().transpose();
  return Eigen::Quaterniond(r_ned_frd).normalized();
}

}  // namespace px4_ctrl_ros2
