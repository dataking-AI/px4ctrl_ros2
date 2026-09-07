#include "px4_ctrl_ros2/controller.hpp"
#include "control_behavior.hpp"
#include "flight_state.hpp"
#include "thrust_estimation_input.hpp"
#include "vehicle_command_authorization.hpp"

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/input_rc.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace px4_ctrl_ros2
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRcDeadZone = 0.25;
constexpr double kHoverModeThreshold = 0.75;
constexpr double kCommandModeThreshold = 0.75;
constexpr double kRebootThreshold = 0.5;
constexpr uint8_t kTakeoffCommand = 1;
constexpr uint8_t kLandCommand = 2;
constexpr double kMinOdomDt = 1.0e-3;

double apply_dead_zone(double value)
{
  value = std::clamp(value, -1.0, 1.0);
  if (value > kRcDeadZone) {
    return (value - kRcDeadZone) / (1.0 - kRcDeadZone);
  }
  if (value < -kRcDeadZone) {
    return (value + kRcDeadZone) / (1.0 - kRcDeadZone);
  }
  return 0.0;
}

double pwm_to_norm(uint16_t value)
{
  return std::clamp((static_cast<double>(value) - 1500.0) / 500.0, -1.0, 1.0);
}

double pwm_to_switch(uint16_t value)
{
  return std::clamp((static_cast<double>(value) - 1000.0) / 1000.0, 0.0, 1.0);
}

double aux_to_switch(double value)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  if (value < -0.05) {
    return std::clamp((value + 1.0) * 0.5, 0.0, 1.0);
  }
  return std::clamp(value, 0.0, 1.0);
}

std::array<float, 4> eigen_quat_to_px4_array(const Eigen::Quaterniond &q)
{
  return {
    static_cast<float>(q.w()),
    static_cast<float>(q.x()),
    static_cast<float>(q.y()),
    static_cast<float>(q.z())};
}

geometry_msgs::msg::Quaternion eigen_to_msg(const Eigen::Quaterniond &q)
{
  geometry_msgs::msg::Quaternion msg{};
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  return msg;
}

bool finite_vector3(const Eigen::Vector3d &v)
{
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}
}  // namespace

struct RcState
{
  std::array<double, 4> ch{{0.0, 0.0, 0.0, 0.0}};
  double mode{0.0};
  double gear{0.0};
  double reboot_cmd{0.0};
  double last_mode{0.0};
  double last_gear{0.0};
  double last_reboot_cmd{0.0};
  bool have_last{false};
  bool is_hover_mode{false};
  bool enter_hover_mode{false};
  bool is_command_mode{false};
  bool enter_command_mode{false};
  bool toggle_reboot{false};

  void update_edges()
  {
    if (!have_last) {
      last_mode = mode;
      last_gear = gear;
      last_reboot_cmd = reboot_cmd;
      have_last = true;
    }

    enter_hover_mode = last_mode < kHoverModeThreshold && mode > kHoverModeThreshold;
    is_hover_mode = mode > kHoverModeThreshold;
    enter_command_mode = is_hover_mode && last_gear < kCommandModeThreshold && gear > kCommandModeThreshold;
    is_command_mode = is_hover_mode && gear > kCommandModeThreshold;
    toggle_reboot = !is_hover_mode && !is_command_mode &&
      last_reboot_cmd < kRebootThreshold && reboot_cmd > kRebootThreshold;

    last_mode = mode;
    last_gear = gear;
    last_reboot_cmd = reboot_cmd;
  }

  bool check_centered() const
  {
    return std::abs(ch[0]) < 1.0e-5 &&
      std::abs(ch[1]) < 1.0e-5 &&
      std::abs(ch[2]) < 1.0e-5 &&
      std::abs(ch[3]) < 1.0e-5;
  }
};

class Px4CtrlNode : public rclcpp::Node
{
public:
  Px4CtrlNode() : Node("px4_ctrl_node"), controller_(read_control_params())
  {
    read_runtime_params();
    setup_ros_interfaces();

    const double frequency = std::max(1.0, params_.ctrl_freq_max);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / frequency));
    timer_ = create_wall_timer(period, std::bind(&Px4CtrlNode::control_loop, this));
    RCLCPP_INFO(
      get_logger(),
      "[px4_ctrl_ros2] started | freq=%.1fHz output=%s bodyrate=%s rc_required=%s",
      frequency,
      yes_no(enable_offboard_command_),
      yes_no(params_.use_bodyrate_ctrl),
      yes_no(rc_required_));
  }

private:
  using BatteryStatus = px4_msgs::msg::BatteryStatus;
  using InputRc = px4_msgs::msg::InputRc;
  using ManualControlSetpoint = px4_msgs::msg::ManualControlSetpoint;
  using OffboardControlMode = px4_msgs::msg::OffboardControlMode;
  using Odometry = nav_msgs::msg::Odometry;
  using PositionCommand = quadrotor_msgs::msg::PositionCommand;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using UInt8 = std_msgs::msg::UInt8;
  using VehicleAttitudeSetpoint = px4_msgs::msg::VehicleAttitudeSetpoint;
  using VehicleCommand = px4_msgs::msg::VehicleCommand;
  using VehicleLandDetected = px4_msgs::msg::VehicleLandDetected;
  using VehicleOdometry = px4_msgs::msg::VehicleOdometry;
  using VehicleRatesSetpoint = px4_msgs::msg::VehicleRatesSetpoint;
  using VehicleStatus = px4_msgs::msg::VehicleStatus;
  using SensorCombined = px4_msgs::msg::SensorCombined;

  ControlParams params_{};
  Controller controller_;
  RcState rc_{};
  OdomState odom_{};
  DesiredState planner_des_{};
  DesiredState exit_hold_des_{};
  PositionCommand planner_cmd_{};
  VehicleStatus vehicle_status_{};
  VehicleLandDetected land_detected_{};
  Eigen::Vector3d imu_acc_flu_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond fcu_attitude_nwu_flu_{Eigen::Quaterniond::Identity()};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_pub_;
  rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr rates_setpoint_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr planner_trigger_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr debug_odom_pub_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr nav_odom_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_fallback_sub_;
  rclcpp::Subscription<ManualControlSetpoint>::SharedPtr manual_control_sub_;
  rclcpp::Subscription<InputRc>::SharedPtr input_rc_sub_;
  rclcpp::Subscription<BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<VehicleLandDetected>::SharedPtr land_detected_sub_;
  rclcpp::Subscription<SensorCombined>::SharedPtr sensor_combined_sub_;
  rclcpp::Subscription<PositionCommand>::SharedPtr planner_cmd_sub_;
  rclcpp::Subscription<UInt8>::SharedPtr takeoff_land_sub_;

  FlightState state_{FlightState::MANUAL_CTRL};
  rclcpp::Time state_enter_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_rc_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_battery_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_land_detected_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_fcu_attitude_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_setpoint_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_nav_odom_callback_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time planner_trigger_not_before_{0, 0, RCL_ROS_TIME};
  rclcpp::Time offboard_exit_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_offboard_exit_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_offboard_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_arm_request_time_{0, 0, RCL_ROS_TIME};

  bool have_odom_{false};
  bool have_status_{false};
  bool have_rc_{false};
  bool have_battery_{false};
  bool have_land_detected_{false};
  bool have_imu_{false};
  bool have_fcu_attitude_{false};
  bool imu_accelerometer_clipped_{false};
  bool have_cmd_{false};
  bool have_nav_state_before_offboard_{false};
  bool planner_trigger_sent_for_command_{false};
  bool offboard_requested_{false};
  bool hover_stable_started_{false};
  bool rc_required_{true};
  bool enable_offboard_command_{false};
  bool enable_auto_arm_{false};
  bool enable_auto_takeoff_land_{false};
  bool auto_start_planner_{false};
  bool publish_debug_odom_{true};
  bool estimate_nav_odom_velocity_{false};
  bool verbose_{false};
  bool reverse_roll_{false};
  bool reverse_pitch_{false};
  bool reverse_yaw_{false};
  bool reverse_throttle_{true};
  double msg_timeout_odom_{0.5};
  double msg_timeout_status_{2.0};
  double msg_timeout_rc_{0.5};
  double msg_timeout_cmd_{0.5};
  double msg_timeout_bat_{0.5};
  double msg_timeout_imu_{0.5};
  double hover_stable_pos_tol_{0.30};
  double hover_stable_vel_tol_{0.30};
  double hover_stable_time_{1.0};
  double takeoff_height_{1.0};
  double takeoff_land_speed_{0.14};
  double battery_voltage_{14.0};
  double offboard_exit_timeout_{1.0};
  double offboard_exit_retry_interval_{0.2};
  uint64_t imu_sample_id_{0};
  uint64_t last_consumed_imu_sample_id_{0};
  uint64_t offboard_setpoint_counter_{0};
  uint64_t offboard_control_mode_publish_count_{0};
  uint64_t attitude_setpoint_publish_count_{0};
  uint64_t rates_setpoint_publish_count_{0};
  uint64_t nav_odom_callback_count_{0};
  uint32_t active_planner_traj_id_{0};
  uint32_t completed_planner_traj_id_{0};
  uint8_t takeoff_land_command_{0};
  uint8_t nav_state_before_offboard_{VehicleStatus::NAVIGATION_STATE_POSCTL};
  Eigen::Vector3d hover_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_nav_odom_position_{Eigen::Vector3d::Zero()};
  double hover_yaw_{0.0};
  double nav_odom_last_gap_{-1.0};
  double nav_odom_max_gap_{-1.0};
  double takeoff_spoolup_time_{3.0};
  double takeoff_trigger_delay_{2.0};
  double takeoff_link_loss_timeout_{0.5};
  std::string odom_source_{"px4"};
  std::string odom_frame_id_{"world"};
  std::string odom_child_frame_id_{"base_link"};
  rclcpp::Time takeoff_spool_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time takeoff_link_loss_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_nav_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time hover_stable_start_time_{0, 0, RCL_ROS_TIME};

  ControlParams read_control_params()
  {
    ControlParams params{};
    params.mass = declare_parameter<double>("mass", params.mass);
    params.gra = declare_parameter<double>("gra", params.gra);
    params.pose_solver = declare_parameter<int>("pose_solver", params.pose_solver);
    params.ctrl_freq_max = declare_parameter<double>("ctrl_freq_max", params.ctrl_freq_max);
    params.use_bodyrate_ctrl = declare_parameter<bool>("use_bodyrate_ctrl", params.use_bodyrate_ctrl);
    params.max_manual_vel = declare_parameter<double>("max_manual_vel", params.max_manual_vel);
    const double max_angle_deg = declare_parameter<double>("max_angle", 30.0);
    params.max_angle_rad = max_angle_deg < 0.0 ? -1.0 : max_angle_deg * kPi / 180.0;
    params.low_voltage = declare_parameter<double>("low_voltage", params.low_voltage);
    params.hover_percentage = declare_parameter<double>("hover_percentage", params.hover_percentage);
    params.accurate_thrust_model =
      declare_parameter<bool>("accurate_thrust_model", params.accurate_thrust_model);
    params.thrust_model_print_value =
      declare_parameter<bool>("thrust_model_print_value", params.thrust_model_print_value);
    params.thrust_model_k1 = declare_parameter<double>("thrust_model_k1", params.thrust_model_k1);
    params.thrust_model_k2 = declare_parameter<double>("thrust_model_k2", params.thrust_model_k2);
    params.thrust_model_k3 = declare_parameter<double>("thrust_model_k3", params.thrust_model_k3);
    params.rotor_drag.x() = declare_parameter<double>("rotor_drag_x", 0.0);
    params.rotor_drag.y() = declare_parameter<double>("rotor_drag_y", 0.0);
    params.rotor_drag.z() = declare_parameter<double>("rotor_drag_z", 0.0);
    params.rotor_drag_k_thrust_horz =
      declare_parameter<double>("rotor_drag_k_thrust_horz", params.rotor_drag_k_thrust_horz);
    params.kp = read_vector3_param("kp", params.kp);
    params.kv = read_vector3_param("kv", params.kv);
    params.kang = read_vector3_param("kang", params.kang);
    params.pos_error_limit = declare_parameter<double>("pos_error_limit", params.pos_error_limit);
    params.vel_error_limit = declare_parameter<double>("vel_error_limit", params.vel_error_limit);
    return params;
  }

  void read_runtime_params()
  {
    msg_timeout_odom_ = declare_parameter<double>("msg_timeout_odom", msg_timeout_odom_);
    msg_timeout_status_ = declare_parameter<double>("msg_timeout_status", msg_timeout_status_);
    msg_timeout_rc_ = declare_parameter<double>("msg_timeout_rc", msg_timeout_rc_);
    msg_timeout_cmd_ = declare_parameter<double>("msg_timeout_cmd", msg_timeout_cmd_);
    msg_timeout_bat_ = declare_parameter<double>("msg_timeout_bat", msg_timeout_bat_);
    msg_timeout_imu_ = declare_parameter<double>("msg_timeout_imu", msg_timeout_imu_);
    reverse_roll_ = declare_parameter<bool>("rc_reverse_roll", reverse_roll_);
    reverse_pitch_ = declare_parameter<bool>("rc_reverse_pitch", reverse_pitch_);
    reverse_yaw_ = declare_parameter<bool>("rc_reverse_yaw", reverse_yaw_);
    reverse_throttle_ = declare_parameter<bool>("rc_reverse_throttle", reverse_throttle_);
    rc_required_ = declare_parameter<bool>("rc_required", rc_required_);
    enable_offboard_command_ = declare_parameter<bool>("enable_offboard_command", enable_offboard_command_);
    enable_auto_arm_ = declare_parameter<bool>("enable_auto_arm", enable_auto_arm_);
    enable_auto_takeoff_land_ =
      declare_parameter<bool>("enable_auto_takeoff_land", enable_auto_takeoff_land_);
    auto_start_planner_ = declare_parameter<bool>("auto_start_planner", auto_start_planner_);
    odom_source_ = declare_parameter<std::string>("odom_source", odom_source_);
    if (odom_source_ != "px4" && odom_source_ != "nav") {
      RCLCPP_WARN(
        get_logger(),
        "[px4_ctrl_ros2] unsupported odom_source '%s'; using px4",
        odom_source_.c_str());
      odom_source_ = "px4";
    }
    estimate_nav_odom_velocity_ =
      declare_parameter<bool>("estimate_nav_odom_velocity", estimate_nav_odom_velocity_);
    hover_stable_pos_tol_ = declare_parameter<double>("hover_stable_pos_tol", hover_stable_pos_tol_);
    hover_stable_vel_tol_ = declare_parameter<double>("hover_stable_vel_tol", hover_stable_vel_tol_);
    hover_stable_time_ = declare_parameter<double>("hover_stable_time", hover_stable_time_);
    takeoff_height_ = declare_parameter<double>("takeoff_height", takeoff_height_);
    takeoff_land_speed_ = declare_parameter<double>("takeoff_land_speed", takeoff_land_speed_);
    takeoff_spoolup_time_ = std::max(
      0.0, declare_parameter<double>("takeoff_spoolup_time", takeoff_spoolup_time_));
    takeoff_trigger_delay_ = std::max(
      0.0, declare_parameter<double>("takeoff_trigger_delay", takeoff_trigger_delay_));
    takeoff_link_loss_timeout_ = std::max(
      0.0, declare_parameter<double>("takeoff_link_loss_timeout", takeoff_link_loss_timeout_));
    publish_debug_odom_ = declare_parameter<bool>("publish_debug_odom", publish_debug_odom_);
    verbose_ = declare_parameter<bool>("verbose", verbose_);
    battery_voltage_ = params_.low_voltage;
  }

  Eigen::Vector3d read_vector3_param(const std::string &name, const Eigen::Vector3d &fallback)
  {
    const std::vector<double> values = declare_parameter<std::vector<double>>(
      name, {fallback.x(), fallback.y(), fallback.z()});
    if (values.size() != 3) {
      RCLCPP_WARN(get_logger(), "[px4_ctrl_ros2] parameter %s must have 3 elements; using fallback", name.c_str());
      return fallback;
    }
    return Eigen::Vector3d(values[0], values[1], values[2]);
  }

  void setup_ros_interfaces()
  {
    rmw_qos_profile_t px4_out_qos_profile = rmw_qos_profile_sensor_data;
    auto px4_out_qos = rclcpp::QoS(
      rclcpp::QoSInitialization(px4_out_qos_profile.history, 5),
      px4_out_qos_profile);

    offboard_control_mode_pub_ =
      create_publisher<OffboardControlMode>("px4/in/offboard_control_mode", 10);
    vehicle_command_pub_ = create_publisher<VehicleCommand>("px4/in/vehicle_command", 10);
    attitude_setpoint_pub_ =
      create_publisher<VehicleAttitudeSetpoint>("px4/in/vehicle_attitude_setpoint", 10);
    rates_setpoint_pub_ =
      create_publisher<VehicleRatesSetpoint>("px4/in/vehicle_rates_setpoint", 10);
    planner_trigger_pub_ = create_publisher<PoseStamped>("ego/traj_start_trigger", 10);
    debug_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("px4ctrl/debug_odom_nwu", 10);

    if (odom_source_ == "nav") {
      nav_odom_sub_ = create_subscription<Odometry>(
        "nav/odom",
        rclcpp::QoS(20),
        std::bind(&Px4CtrlNode::nav_odometry_callback, this, std::placeholders::_1));
      vehicle_odometry_sub_ = create_subscription<VehicleOdometry>(
        "px4/out/vehicle_odometry",
        px4_out_qos,
        std::bind(&Px4CtrlNode::vehicle_odometry_callback, this, std::placeholders::_1));
    } else {
      nav_odom_sub_ = create_subscription<Odometry>(
        "px4/odom_nwu",
        rclcpp::QoS(20),
        std::bind(&Px4CtrlNode::nav_odometry_callback, this, std::placeholders::_1));
    }
    vehicle_status_sub_ = create_subscription<VehicleStatus>(
      "px4/out/vehicle_status_v1",
      px4_out_qos,
      std::bind(&Px4CtrlNode::vehicle_status_callback, this, std::placeholders::_1));
    vehicle_status_fallback_sub_ = create_subscription<VehicleStatus>(
      "px4/out/vehicle_status",
      px4_out_qos,
      std::bind(&Px4CtrlNode::vehicle_status_callback, this, std::placeholders::_1));
    manual_control_sub_ = create_subscription<ManualControlSetpoint>(
      "px4/out/manual_control_setpoint",
      px4_out_qos,
      std::bind(&Px4CtrlNode::manual_control_callback, this, std::placeholders::_1));
    input_rc_sub_ = create_subscription<InputRc>(
      "px4/out/input_rc",
      px4_out_qos,
      std::bind(&Px4CtrlNode::input_rc_callback, this, std::placeholders::_1));
    battery_sub_ = create_subscription<BatteryStatus>(
      "px4/out/battery_status",
      px4_out_qos,
      std::bind(&Px4CtrlNode::battery_callback, this, std::placeholders::_1));
    land_detected_sub_ = create_subscription<VehicleLandDetected>(
      "px4/out/vehicle_land_detected",
      px4_out_qos,
        std::bind(&Px4CtrlNode::land_detected_callback, this, std::placeholders::_1));
    sensor_combined_sub_ = create_subscription<SensorCombined>(
      "px4/out/sensor_combined",
      px4_out_qos,
      std::bind(&Px4CtrlNode::sensor_combined_callback, this, std::placeholders::_1));
    planner_cmd_sub_ = create_subscription<PositionCommand>(
      "ego/position_cmd",
      10,
      std::bind(&Px4CtrlNode::planner_cmd_callback, this, std::placeholders::_1));
    takeoff_land_sub_ = create_subscription<UInt8>(
      "px4ctrl/takeoff_land_cmd",
      10,
      std::bind(&Px4CtrlNode::takeoff_land_callback, this, std::placeholders::_1));
  }

  void control_loop()
  {
    publish_odometry_outputs();
    log_diagnostics();

    if (!odom_ready()) {
      if (state_ != FlightState::MANUAL_CTRL) {
        transition_to(FlightState::FAILSAFE, "odometry timeout or invalid");
      }
      return;
    }

    const auto now = get_clock()->now();
    const double dt = last_control_time_.nanoseconds() == 0 ?
      1.0 / std::max(1.0, params_.ctrl_freq_max) :
      std::clamp((now - last_control_time_).seconds(), 0.0, 0.1);
    last_control_time_ = now;

    switch (state_) {
      case FlightState::MANUAL_CTRL:
        handle_manual_state();
        break;
      case FlightState::AUTO_HOVER:
        handle_auto_hover_state(dt);
        break;
      case FlightState::CMD_CTRL:
        handle_cmd_ctrl_state(dt);
        break;
      case FlightState::AUTO_TAKEOFF:
        handle_auto_takeoff_state();
        break;
      case FlightState::AUTO_LAND:
        handle_auto_land_state(dt);
        break;
      case FlightState::EXITING_OFFBOARD:
        handle_exiting_offboard_state();
        break;
      case FlightState::FAILSAFE:
        handle_failsafe_state();
        break;
    }
  }

  void handle_manual_state()
  {
    reset_offboard_requests();
    if (takeoff_land_command_ == kTakeoffCommand && enable_auto_takeoff_land_) {
      if (planner_cmd_received()) {
        RCLCPP_ERROR(
          get_logger(),
          "[px4_ctrl_ros2] Reject AUTO_TAKEOFF. You are sending commands before toggling into AUTO_TAKEOFF, which is not allowed. Stop sending commands now!");
        takeoff_land_command_ = 0;
        return;
      }
      if (odom_.v.norm() > 0.1) {
        RCLCPP_ERROR(
          get_logger(),
          "[px4_ctrl_ros2] Reject AUTO_TAKEOFF. Odom_Vel=%fm/s, non-static takeoff is not allowed!",
          odom_.v.norm());
        takeoff_land_command_ = 0;
        return;
      }
      if (have_land_detected_ && !land_detected_.landed) {
        RCLCPP_ERROR(
          get_logger(),
          "[px4_ctrl_ros2] Reject AUTO_TAKEOFF. land detector says that the drone is not landed now!");
        takeoff_land_command_ = 0;
        return;
      }
      if (rc_required_ && rc_control_available() &&
        (!rc_.is_hover_mode || !rc_.is_command_mode || !rc_.check_centered())) {
        RCLCPP_ERROR(
          get_logger(),
          "[px4_ctrl_ros2] Reject AUTO_TAKEOFF. If you have your RC connected, keep its switches at auto hover and command control states, and all sticks at the center, then takeoff again.");
        takeoff_land_command_ = 0;
        return;
      }

      hover_position_ = odom_.p;
      hover_yaw_ = yaw_from_quaternion(odom_.q);
      takeoff_spool_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      takeoff_link_loss_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      planner_trigger_not_before_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

      planner_trigger_sent_for_command_ = false;
      hover_stable_started_ = false;
      takeoff_land_command_ = 0;
      transition_to(FlightState::AUTO_TAKEOFF, "takeoff command accepted");
      return;
    }

    if (rc_control_available() && rc_.enter_hover_mode) {
      if (planner_cmd_received()) {
        RCLCPP_ERROR(
          get_logger(),
          "[px4_ctrl_ros2] Reject AUTO_HOVER. You are sending commands before toggling into AUTO_HOVER, which is not allowed. Stop sending commands now!");
        return;
      }
      if (odom_.v.norm() > 3.0) {
        RCLCPP_ERROR(
          get_logger(),
          "[px4_ctrl_ros2] Reject AUTO_HOVER. Odom_Vel=%fm/s, which seems that the localization module goes wrong!",
          odom_.v.norm());
        return;
      }

      set_hover_from_current(0.0);
      transition_to(FlightState::AUTO_HOVER, "RC hover switch entered");
    }
  }

  void handle_auto_hover_state(double dt)
  {
    if (!rc_control_allowed()) {
      begin_normal_offboard_exit("RC hover switch released or RC timeout");
      return;
    }

    if (takeoff_land_command_ == kLandCommand && enable_auto_takeoff_land_) {
      takeoff_land_command_ = 0;
      transition_to(FlightState::AUTO_LAND, "land command accepted");
      return;
    }

    update_hover_from_rc(dt);
    publish_control(make_hover_desired());

    const bool trigger_delay_elapsed =
      planner_trigger_not_before_.nanoseconds() == 0 || now() >= planner_trigger_not_before_;
    const bool command_authorized = !rc_required_ || rc_.is_command_mode;
    const bool should_trigger_planner =
      (rc_required_ && rc_.is_command_mode) ||
      (!rc_required_ && auto_start_planner_);
    const bool hover_stable = hover_is_stable();
    if (command_authorized && should_trigger_planner && hover_stable && trigger_delay_elapsed &&
      !planner_trigger_sent_for_command_)
    {
      trigger_planner_once();
    }

    if (command_authorized && vehicle_is_offboard() && planner_cmd_ready()) {
      active_planner_traj_id_ = planner_cmd_.trajectory_id;
      transition_to(FlightState::CMD_CTRL, "fresh PositionCommand with RC command authorization");
      return;
    }
  }

  void handle_cmd_ctrl_state(double dt)
  {
    if (takeoff_land_command_ == kLandCommand && enable_auto_takeoff_land_) {
      RCLCPP_ERROR(
        get_logger(),
        "[px4_ctrl_ros2] Reject AUTO_LAND, which must be triggered in AUTO_HOVER. Stop sending control commands for longer than %.3fs to let px4ctrl return to AUTO_HOVER first.",
        msg_timeout_cmd_);
      takeoff_land_command_ = 0;
    }

    if (!rc_control_allowed()) {
      begin_normal_offboard_exit("RC hover switch released or RC timeout");
      return;
    }

    if (rc_required_ && !rc_.is_command_mode) {
      transition_to(FlightState::AUTO_HOVER, "RC command switch released");
      return;
    }
    if (planner_cmd_completed()) {
      completed_planner_traj_id_ = planner_cmd_.trajectory_id;
      set_hover_from_current(0.0);
      transition_to(FlightState::AUTO_HOVER, "planner trajectory completed");
      return;
    }
    if (!planner_cmd_ready()) {
      set_hover_from_current(0.0);
      transition_to(FlightState::AUTO_HOVER, "PositionCommand timeout or invalid");
      return;
    }

    update_hover_from_rc(dt);
    publish_control(planner_des_);
  }

  void handle_auto_takeoff_state()
  {
    if (rc_required_ && (!rc_control_available() || !rc_.is_hover_mode)) {
      begin_normal_offboard_exit("operator aborted takeoff");
      return;
    }

    if (takeoff_spool_start_time_.nanoseconds() == 0) {
      publish_control(make_takeoff_spool_desired(0.0));
      if (vehicle_is_offboard() && vehicle_is_armed()) {
        takeoff_spool_start_time_ = now();
      }
      return;
    }

    if (!vehicle_is_offboard() || !vehicle_is_armed()) {
      // PX4 publishes vehicle_status at 2 Hz (Commander.cpp publishes it every
      // 500 ms or on change), so a single stale sample must not be treated as a
      // real loss of Offboard or armed state.
      const auto current_time = now();
      if (takeoff_link_loss_start_time_.nanoseconds() == 0) {
        takeoff_link_loss_start_time_ = current_time;
      }
      const double lost_seconds = (current_time - takeoff_link_loss_start_time_).seconds();
      if (lost_seconds >= takeoff_link_loss_timeout_) {
        transition_to(FlightState::FAILSAFE, "Offboard or armed state lost during takeoff");
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 500,
          "[px4_ctrl_ros2] offboard/armed link lost for %.2fs during takeoff; waiting for recovery",
          lost_seconds);
      }
      return;
    }
    takeoff_link_loss_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    const double elapsed = (now() - takeoff_spool_start_time_).seconds();

    if (elapsed < takeoff_spoolup_time_) {
      publish_control(make_takeoff_spool_desired(elapsed));
      return;
    }

    if (odom_.p.z() >= hover_position_.z() + takeoff_height_) {
      set_hover_from_current(0.0);
      planner_trigger_not_before_ =
        now() + rclcpp::Duration::from_seconds(takeoff_trigger_delay_);
      transition_to(FlightState::AUTO_HOVER, "takeoff height reached");
      return;
    }

    publish_control(make_takeoff_desired(elapsed));
  }

  void handle_auto_land_state(double dt)
  {
    if (!odom_ready()) {
      transition_to(FlightState::FAILSAFE, "odometry lost during land");
      return;
    }

    switch (auto_land_rc_action(
        rc_required_, rc_control_available(), rc_.is_hover_mode, rc_.is_command_mode))
    {
      case AutoLandRcAction::EXIT_OFFBOARD:
        begin_normal_offboard_exit("RC hover switch released or RC timeout during land");
        return;
      case AutoLandRcAction::HOLD_POSITION:
        set_hover_from_current(0.0);
        transition_to(FlightState::AUTO_HOVER, "RC command switch released during land");
        return;
      case AutoLandRcAction::CONTINUE_LANDING:
        break;
    }

    hover_position_.z() = std::max(0.0, hover_position_.z() - takeoff_land_speed_ * dt);
    publish_control(make_hover_desired());

    if (have_land_detected_ && land_detected_.landed) {
      if (vehicle_is_armed()) {
        if (!enable_auto_arm_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "[px4_ctrl_ros2] landed but still armed; waiting for manual disarm");
          return;
        }

        const auto current_time = now();
        if (last_arm_request_time_.nanoseconds() == 0 ||
          (current_time - last_arm_request_time_).seconds() > 1.0)
        {
          if (publish_vehicle_command(
              VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f))
          {
            last_arm_request_time_ = current_time;
            RCLCPP_INFO(get_logger(), "[px4_ctrl_ros2] DISARM command sent");
          }
        }
        return;
      }
      begin_normal_offboard_exit("landed and disarmed");
    }
  }

  void handle_failsafe_state()
  {
    reset_offboard_requests();
    if (rc_control_available() && !rc_.is_hover_mode) {
      transition_to(FlightState::MANUAL_CTRL, "RC returned to manual");
    }
  }

  DesiredState make_hover_desired() const
  {
    DesiredState des{};
    des.p = hover_position_;
    des.v.setZero();
    des.a.setZero();
    des.j.setZero();
    des.yaw = hover_yaw_;
    des.yaw_rate = 0.0;
    return des;
  }

  DesiredState make_takeoff_spool_desired(double elapsed) const
  {
    DesiredState des = make_hover_desired();
    const double t = std::clamp(elapsed, 0.0, takeoff_spoolup_time_);
    des.a.z() = std::min(
      std::exp((t - takeoff_spoolup_time_) * 6.0) * 7.0 - 7.0,
      0.0);
    return des;
  }

  DesiredState make_takeoff_desired(double elapsed) const
  {
    DesiredState des = make_hover_desired();
    const double ascend_time = std::max(0.0, elapsed - takeoff_spoolup_time_);
    des.p.z() += takeoff_land_speed_ * ascend_time;
    des.v.z() = takeoff_land_speed_;
    return des;
  }

  void publish_control(const DesiredState &des)
  {
    if (!enable_offboard_command_) {
      return;
    }

    const auto command_time = now();
    if ((state_ == FlightState::AUTO_HOVER || state_ == FlightState::CMD_CTRL) &&
      imu_sample_is_usable(command_time))
    {
      last_consumed_imu_sample_id_ = imu_sample_id_;
      const bool thrust_model_updated =
        controller_.estimate_thrust_model(imu_acc_flu_.z(), last_imu_time_);
      if (thrust_model_updated && !params_.accurate_thrust_model &&
        params_.thrust_model_print_value)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "[px4_ctrl_ros2] hover_percentage=%.3f thr2acc=%.3f",
          controller_.debug().hover_percentage,
          controller_.debug().thr2acc);
      }
      if (thrust_model_updated && controller_.debug().thrust_estimate_clamped) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "[px4_ctrl_ros2] online thrust estimate exceeded safe hover range; clamped hover_percentage=%.3f thr2acc=%.3f",
          controller_.debug().hover_percentage,
          controller_.debug().thr2acc);
      }
    }

    auto output = controller_.update_alg1(des, odom_, battery_voltage_, command_time);
    if (!control_output_is_finite(output)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[px4_ctrl_ros2] controller output is non-finite; skip setpoint");
      return;
    }
    if (odom_source_ == "nav" && !params_.use_bodyrate_ctrl) {
      const double fcu_attitude_age = (command_time - last_fcu_attitude_time_).seconds();
      if (!have_fcu_attitude_ || !std::isfinite(fcu_attitude_age) ||
        fcu_attitude_age < 0.0 || fcu_attitude_age >= msg_timeout_odom_)
      {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[px4_ctrl_ros2] nav odometry requires a fresh PX4 FCU attitude; skip setpoint");
        return;
      }
      output.q = align_nav_attitude_to_fcu(fcu_attitude_nwu_flu_, odom_.q, output.q);
    }

    // Publish the Offboard heartbeat only when a matching, valid setpoint is
    // ready. If any prerequisite above fails, withholding the heartbeat lets
    // PX4's Offboard-loss failsafe take over instead of advertising a live
    // control link while leaving PX4 with a stale attitude/rate setpoint.
    publish_offboard_control_mode();

    if (params_.use_bodyrate_ctrl) {
      publish_rates_setpoint(output);
    } else {
      publish_attitude_setpoint(output);
    }

    offboard_setpoint_counter_++;
    maybe_request_offboard_and_arm();
  }

  void publish_offboard_control_mode()
  {
    OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = !params_.use_bodyrate_ctrl;
    msg.body_rate = params_.use_bodyrate_ctrl;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    msg.timestamp = timestamp_us();
    offboard_control_mode_pub_->publish(msg);
    offboard_control_mode_publish_count_++;
  }

  void publish_attitude_setpoint(const ControllerOutput &output)
  {
    const Eigen::Quaterniond q_ned_frd = nwu_flu_to_ned_frd(output.q);
    VehicleAttitudeSetpoint msg{};
    msg.q_d = eigen_quat_to_px4_array(q_ned_frd);
    msg.thrust_body = {0.0f, 0.0f, static_cast<float>(-output.thrust)};
    msg.yaw_sp_move_rate = attitude_yaw_sp_move_rate();
    msg.reset_integral = false;
    msg.fw_control_yaw_wheel = false;
    msg.timestamp = timestamp_us();
    attitude_setpoint_pub_->publish(msg);
    attitude_setpoint_publish_count_++;
    last_setpoint_publish_time_ = now();
  }

  void publish_rates_setpoint(const ControllerOutput &output)
  {
    VehicleRatesSetpoint msg{};
    msg.roll = static_cast<float>(output.bodyrates.x());
    msg.pitch = static_cast<float>(-output.bodyrates.y());
    msg.yaw = static_cast<float>(-output.bodyrates.z());
    msg.thrust_body = {0.0f, 0.0f, static_cast<float>(-output.thrust)};
    msg.reset_integral = false;
    msg.timestamp = timestamp_us();
    rates_setpoint_pub_->publish(msg);
    rates_setpoint_publish_count_++;
    last_setpoint_publish_time_ = now();
  }

  void maybe_request_offboard_and_arm()
  {
    if (offboard_setpoint_counter_ < 10) {
      return;
    }

    const auto current_time = now();
    const bool arm_request_interval_elapsed =
      last_arm_request_time_.nanoseconds() == 0 ||
      (current_time - last_arm_request_time_).seconds() > 1.0;
    if (!vehicle_is_offboard() &&
      (last_offboard_request_time_.nanoseconds() == 0 ||
      (current_time - last_offboard_request_time_).seconds() > 1.0)) {
      if (!offboard_requested_ && status_ready()) {
        nav_state_before_offboard_ = vehicle_status_.nav_state;
        have_nav_state_before_offboard_ = true;
      }
      const bool first_request = !offboard_requested_;
      if (publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f)) {
        offboard_requested_ = true;
        last_offboard_request_time_ = current_time;
        if (first_request) {
          RCLCPP_INFO(get_logger(), "[px4_ctrl_ros2] OFFBOARD mode command sent");
        } else {
          RCLCPP_DEBUG(get_logger(), "[px4_ctrl_ros2] OFFBOARD mode command retried");
        }
      }
    }

    if (enable_auto_arm_ &&
      state_ == FlightState::AUTO_TAKEOFF &&
      vehicle_is_offboard() &&
      !vehicle_is_armed() &&
      arm_request_interval_elapsed) {
      const bool first_request = last_arm_request_time_.nanoseconds() == 0;
      if (publish_vehicle_command(
          VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f))
      {
        last_arm_request_time_ = current_time;
        if (first_request) {
          RCLCPP_INFO(get_logger(), "[px4_ctrl_ros2] ARM command sent");
        } else {
          RCLCPP_DEBUG(get_logger(), "[px4_ctrl_ros2] ARM command retried");
        }
      }
    }
  }

  bool publish_vehicle_command(uint32_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    const auto authority = command == VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM ?
      VehicleCommandAuthority::ARM_DISARM : VehicleCommandAuthority::OFFBOARD;
    if (!vehicle_command_allowed(
        authority, enable_offboard_command_, enable_auto_arm_))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "[px4_ctrl_ros2] vehicle command %u blocked by safety configuration",
        command);
      return false;
    }

    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = timestamp_us();
    vehicle_command_pub_->publish(msg);
    return true;
  }

  void trigger_planner_once()
  {
    if (planner_trigger_sent_for_command_) {
      return;
    }
    PoseStamped msg{};
    msg.header.stamp = now();
    msg.header.frame_id = "world";
    msg.pose.position.x = odom_.p.x();
    msg.pose.position.y = odom_.p.y();
    msg.pose.position.z = odom_.p.z();
    msg.pose.orientation.w = 1.0;
    planner_trigger_pub_->publish(msg);
    planner_trigger_sent_for_command_ = true;
    RCLCPP_INFO(get_logger(), "[px4_ctrl_ros2] planner trigger published");
  }

  void set_hover_from_current(double z_offset)
  {
    hover_position_ = odom_.p;
    hover_position_.z() += z_offset;
    hover_yaw_ = yaw_from_quaternion(odom_.q);
    planner_trigger_sent_for_command_ = false;
    planner_trigger_not_before_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    hover_stable_started_ = false;
    active_planner_traj_id_ = 0;
  }

  void set_hover_from_desired(const DesiredState &des)
  {
    if (finite_vector3(des.p)) {
      hover_position_ = des.p;
    }
    hover_yaw_ = des.yaw;
  }

  void update_hover_from_rc(double dt)
  {
    if (!rc_control_available()) {
      return;
    }

    const double roll = reverse_roll_ ? -rc_.ch[0] : rc_.ch[0];
    const double pitch = reverse_pitch_ ? -rc_.ch[1] : rc_.ch[1];
    const double throttle = reverse_throttle_ ? -rc_.ch[2] : rc_.ch[2];
    const double yaw = reverse_yaw_ ? -rc_.ch[3] : rc_.ch[3];

    hover_position_.x() += pitch * params_.max_manual_vel * dt;
    hover_position_.y() += roll * params_.max_manual_vel * dt;
    hover_position_.z() += throttle * params_.max_manual_vel * dt;
    hover_position_.z() = std::max(0.0, hover_position_.z());
    hover_yaw_ = normalize_angle(hover_yaw_ + yaw * params_.max_manual_vel * dt);
  }

  void reset_offboard_requests()
  {
    offboard_setpoint_counter_ = 0;
    offboard_requested_ = false;
    last_offboard_request_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_arm_request_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    planner_trigger_sent_for_command_ = false;
    hover_stable_started_ = false;
  }

  bool nav_state_to_px4_main_mode(uint8_t nav_state, float &main_mode) const
  {
    switch (nav_state) {
      case VehicleStatus::NAVIGATION_STATE_MANUAL: main_mode = 1.0f; return true;
      case VehicleStatus::NAVIGATION_STATE_ALTCTL: main_mode = 2.0f; return true;
      case VehicleStatus::NAVIGATION_STATE_POSCTL: main_mode = 3.0f; return true;
      case VehicleStatus::NAVIGATION_STATE_ACRO:   main_mode = 5.0f; return true;
      case VehicleStatus::NAVIGATION_STATE_STAB:   main_mode = 7.0f; return true;
      default: return false;
    }
  }

  void begin_normal_offboard_exit(const char *reason)
  {
    if (!enable_offboard_command_) {
      transition_to(FlightState::MANUAL_CTRL, "Offboard command output disabled");
      return;
    }

    exit_hold_des_.p = odom_.p;
    exit_hold_des_.v.setZero();
    exit_hold_des_.a.setZero();
    exit_hold_des_.j.setZero();
    exit_hold_des_.yaw = yaw_from_quaternion(odom_.q);
    exit_hold_des_.yaw_rate = 0.0;

    offboard_exit_start_time_ = now();
    last_offboard_exit_request_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    transition_to(FlightState::EXITING_OFFBOARD, reason);
  }

  void handle_exiting_offboard_state()
  {
    if (!status_ready()) {
      transition_to(FlightState::FAILSAFE, "PX4 status lost during Offboard exit");
      return;
    }

    if (vehicle_status_.nav_state != VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
      have_nav_state_before_offboard_ = false;
      transition_to(FlightState::MANUAL_CTRL, "PX4 confirmed Offboard exit");
      return;
    }

    publish_control(exit_hold_des_);

    const auto current_time = now();
    if (last_offboard_exit_request_time_.nanoseconds() == 0 ||
      (current_time - last_offboard_exit_request_time_).seconds() >=
      offboard_exit_retry_interval_)
    {
      float main_mode = 0.0f;
      if (!have_nav_state_before_offboard_ ||
        !nav_state_to_px4_main_mode(nav_state_before_offboard_, main_mode))
      {
        main_mode = 3.0f;
      }

      if (publish_vehicle_command(
          VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, main_mode))
      {
        last_offboard_exit_request_time_ = current_time;
      }
    }

    if ((current_time - offboard_exit_start_time_).seconds() > offboard_exit_timeout_) {
      transition_to(FlightState::FAILSAFE, "Offboard exit confirmation timeout");
    }
  }

  bool odom_ready()
  {
    return have_odom_ &&
      (now() - last_odom_time_).seconds() < msg_timeout_odom_ &&
      finite_vector3(odom_.p) &&
      finite_vector3(odom_.v) &&
      std::isfinite(odom_.q.w()) &&
      std::isfinite(odom_.q.x()) &&
      std::isfinite(odom_.q.y()) &&
      std::isfinite(odom_.q.z());
  }

  bool status_ready()
  {
    return have_status_ && (now() - last_status_time_).seconds() < msg_timeout_status_;
  }

  bool rc_control_available()
  {
    return have_rc_ && (now() - last_rc_time_).seconds() < msg_timeout_rc_;
  }

  bool rc_control_allowed()
  {
    if (!rc_required_) {
      return true;
    }
    return rc_control_available() && rc_.is_hover_mode;
  }

  bool planner_cmd_ready()
  {
    return planner_cmd_received() &&
      position_cmd_is_trackable(planner_cmd_) &&
      planner_cmd_.trajectory_id > 0 &&
      planner_cmd_.trajectory_id != completed_planner_traj_id_;
  }

  bool planner_cmd_received()
  {
    return have_cmd_ && (now() - last_cmd_time_).seconds() < msg_timeout_cmd_;
  }

  bool planner_cmd_completed()
  {
    return have_cmd_ &&
      (now() - last_cmd_time_).seconds() < msg_timeout_cmd_ &&
      planner_cmd_.trajectory_flag == PositionCommand::TRAJECTORY_STATUS_COMPLETED &&
      planner_cmd_.trajectory_id == active_planner_traj_id_;
  }

  bool control_output_is_finite(const ControllerOutput &output) const
  {
    return std::isfinite(output.q.w()) &&
      std::isfinite(output.q.x()) &&
      std::isfinite(output.q.y()) &&
      std::isfinite(output.q.z()) &&
      finite_vector3(output.bodyrates) &&
      std::isfinite(output.thrust);
  }

  bool vehicle_is_offboard()
  {
    return status_ready() && vehicle_status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }

  bool vehicle_is_armed()
  {
    return status_ready() && vehicle_status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;
  }

  bool imu_sample_is_usable(const rclcpp::Time &sample_time) const
  {
    return thrust_estimation_imu_sample_is_usable(
      have_imu_,
      imu_accelerometer_clipped_,
      imu_acc_flu_,
      imu_sample_id_,
      last_consumed_imu_sample_id_,
      sample_time,
      last_imu_time_,
      msg_timeout_imu_);
  }

  bool position_cmd_is_trackable(const PositionCommand &msg) const
  {
    return msg.trajectory_flag == PositionCommand::TRAJECTORY_STATUS_READY &&
      std::isfinite(msg.position.x) &&
      std::isfinite(msg.position.y) &&
      std::isfinite(msg.position.z) &&
      std::isfinite(msg.velocity.x) &&
      std::isfinite(msg.velocity.y) &&
      std::isfinite(msg.velocity.z) &&
      std::isfinite(msg.acceleration.x) &&
      std::isfinite(msg.acceleration.y) &&
      std::isfinite(msg.acceleration.z) &&
      std::isfinite(msg.yaw) &&
      std::isfinite(msg.yaw_dot);
  }

  bool hover_is_stable()
  {
    if (!odom_ready()) {
      hover_stable_started_ = false;
      return false;
    }

    if (enable_offboard_command_ && (!vehicle_is_offboard() || !vehicle_is_armed())) {
      hover_stable_started_ = false;
      return false;
    }

    const double position_error = (odom_.p - hover_position_).norm();
    const double velocity_norm = odom_.v.norm();
    if (position_error > hover_stable_pos_tol_ || velocity_norm > hover_stable_vel_tol_) {
      hover_stable_started_ = false;
      return false;
    }

    const auto current_time = now();
    if (!hover_stable_started_) {
      hover_stable_started_ = true;
      hover_stable_start_time_ = current_time;
      return false;
    }

    return (current_time - hover_stable_start_time_).seconds() >= hover_stable_time_;
  }

  void transition_to(FlightState next, const char *reason)
  {
    if (state_ == next) {
      return;
    }
    const auto old = state_;
    if (should_reset_thrust_mapping(old, next)) {
      controller_.reset_thrust_mapping();
    }
    state_ = next;
    state_enter_time_ = now();
    if (next == FlightState::MANUAL_CTRL || next == FlightState::FAILSAFE) {
      reset_offboard_requests();
    }
    RCLCPP_INFO(
      get_logger(),
      "\033[32m[px4_ctrl_ros2] %s -> %s | %s\033[0m",
      state_name(old),
      state_name(next),
      reason);
  }

  void vehicle_odometry_callback(const VehicleOdometry::SharedPtr msg)
  {
    if (msg->pose_frame != VehicleOdometry::POSE_FRAME_NED) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "[px4_ctrl_ros2] FCU attitude pose frame is not NED; pose_frame=%u",
        static_cast<unsigned>(msg->pose_frame));
      return;
    }

    const Eigen::Quaterniond q_ned_frd(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    if (!std::isfinite(q_ned_frd.w()) || !std::isfinite(q_ned_frd.x()) ||
      !std::isfinite(q_ned_frd.y()) || !std::isfinite(q_ned_frd.z())) {
      return;
    }
    if (q_ned_frd.norm() < 1.0e-6) {
      return;
    }

    fcu_attitude_nwu_flu_ = ned_frd_to_nwu_flu(q_ned_frd.normalized());
    have_fcu_attitude_ = true;
    last_fcu_attitude_time_ = now();
  }

  void nav_odometry_callback(const Odometry::SharedPtr msg)
  {
    const Eigen::Vector3d p(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);
    Eigen::Vector3d v(
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z);
    const Eigen::Quaterniond q(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);
    const Eigen::Vector3d w(
      msg->twist.twist.angular.x,
      msg->twist.twist.angular.y,
      msg->twist.twist.angular.z);

    if (!finite_vector3(p) || !finite_vector3(v) || !finite_vector3(w) ||
      !std::isfinite(q.w()) || !std::isfinite(q.x()) ||
      !std::isfinite(q.y()) || !std::isfinite(q.z()) ||
      q.norm() < 1.0e-6) {
      return;
    }

    const auto callback_time = now();
    if (last_nav_odom_callback_time_.nanoseconds() != 0) {
      nav_odom_last_gap_ = (callback_time - last_nav_odom_callback_time_).seconds();
      nav_odom_max_gap_ = std::max(nav_odom_max_gap_, nav_odom_last_gap_);
    }
    last_nav_odom_callback_time_ = callback_time;
    nav_odom_callback_count_++;

    const rclcpp::Time msg_time =
      msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
      now() : rclcpp::Time(msg->header.stamp);
    if (estimate_nav_odom_velocity_ && have_odom_) {
      const double dt = (msg_time - last_nav_odom_stamp_).seconds();
      if (dt > kMinOdomDt) {
        v = (p - last_nav_odom_position_) / dt;
      }
    }

    odom_.p = p;
    odom_.v = v;
    odom_.q = q.normalized();
    odom_.w = w;
    have_odom_ = true;
    last_odom_time_ = callback_time;
    last_nav_odom_stamp_ = msg_time;
    last_nav_odom_position_ = p;
    odom_frame_id_ = msg->header.frame_id.empty() ? "world" : msg->header.frame_id;
    odom_child_frame_id_ = msg->child_frame_id.empty() ? "base_link" : msg->child_frame_id;
  }

  void vehicle_status_callback(const VehicleStatus::SharedPtr msg)
  {
    vehicle_status_ = *msg;
    have_status_ = true;
    last_status_time_ = now();
  }

  void manual_control_callback(const ManualControlSetpoint::SharedPtr msg)
  {
    if (!msg->valid) {
      return;
    }
    rc_.ch[0] = apply_dead_zone(msg->roll);
    rc_.ch[1] = apply_dead_zone(msg->pitch);
    rc_.ch[2] = apply_dead_zone(msg->throttle);
    rc_.ch[3] = apply_dead_zone(msg->yaw);
    rc_.mode = aux_to_switch(msg->aux1);
    rc_.gear = aux_to_switch(msg->aux2);
    rc_.reboot_cmd = aux_to_switch(msg->aux4);
    rc_.update_edges();
    have_rc_ = true;
    last_rc_time_ = now();
  }

  void input_rc_callback(const InputRc::SharedPtr msg)
  {
    if (msg->rc_lost || msg->rc_failsafe || msg->channel_count < 8) {
      return;
    }

    rc_.ch[0] = apply_dead_zone(pwm_to_norm(msg->values[0]));
    rc_.ch[1] = apply_dead_zone(pwm_to_norm(msg->values[1]));
    rc_.ch[2] = apply_dead_zone(pwm_to_norm(msg->values[2]));
    rc_.ch[3] = apply_dead_zone(pwm_to_norm(msg->values[3]));
    rc_.mode = pwm_to_switch(msg->values[4]);
    rc_.gear = pwm_to_switch(msg->values[5]);
    rc_.reboot_cmd = pwm_to_switch(msg->values[7]);
    rc_.update_edges();
    have_rc_ = true;
    last_rc_time_ = now();

    if (rc_.toggle_reboot) {
      RCLCPP_WARN(get_logger(), "[px4_ctrl_ros2] RC reboot switch toggled; reboot command is not sent in this port");
    }
  }

  void battery_callback(const BatteryStatus::SharedPtr msg)
  {
    if (msg->connected && std::isfinite(msg->voltage_v) && msg->voltage_v > 1.0) {
      battery_voltage_ = msg->voltage_v;
      have_battery_ = true;
      last_battery_time_ = now();
    }
  }

  void land_detected_callback(const VehicleLandDetected::SharedPtr msg)
  {
    land_detected_ = *msg;
    have_land_detected_ = true;
    last_land_detected_time_ = now();
  }

  void sensor_combined_callback(const SensorCombined::SharedPtr msg)
  {
    const auto acceleration_timestamp_us = sensor_acceleration_timestamp_us(
      msg->timestamp,
      msg->accelerometer_timestamp_relative);
    const auto acceleration_flu = sensor_acceleration_frd_to_flu(msg->accelerometer_m_s2);
    if (!acceleration_timestamp_us.has_value() || !acceleration_flu.has_value()) {
      return;
    }

    imu_acc_flu_ = *acceleration_flu;
    imu_sample_id_ = *acceleration_timestamp_us;
    imu_accelerometer_clipped_ = msg->accelerometer_clipping != 0;
    have_imu_ = true;
    last_imu_time_ = now();
  }

  void planner_cmd_callback(const PositionCommand::SharedPtr msg)
  {
    planner_cmd_ = *msg;
    planner_des_.p = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    planner_des_.v = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    planner_des_.a = Eigen::Vector3d(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
    planner_des_.j.setZero();
    planner_des_.yaw = normalize_angle(msg->yaw);
    planner_des_.yaw_rate = msg->yaw_dot;
    have_cmd_ = true;
    last_cmd_time_ = now();
  }

  void takeoff_land_callback(const UInt8::SharedPtr msg)
  {
    if (msg->data != kTakeoffCommand && msg->data != kLandCommand) {
      RCLCPP_WARN(get_logger(), "[px4_ctrl_ros2] ignore unknown takeoff_land command: %u", msg->data);
      return;
    }
    takeoff_land_command_ = msg->data;
    RCLCPP_INFO(get_logger(), "[px4_ctrl_ros2] takeoff_land command received: %u", msg->data);
  }

  nav_msgs::msg::Odometry make_odom_msg()
  {
    nav_msgs::msg::Odometry msg{};
    msg.header.stamp = now();
    msg.header.frame_id = odom_frame_id_;
    msg.child_frame_id = odom_child_frame_id_;
    msg.pose.pose.position.x = odom_.p.x();
    msg.pose.pose.position.y = odom_.p.y();
    msg.pose.pose.position.z = odom_.p.z();
    msg.pose.pose.orientation = eigen_to_msg(odom_.q);
    msg.twist.twist.linear.x = odom_.v.x();
    msg.twist.twist.linear.y = odom_.v.y();
    msg.twist.twist.linear.z = odom_.v.z();
    msg.twist.twist.angular.x = odom_.w.x();
    msg.twist.twist.angular.y = odom_.w.y();
    msg.twist.twist.angular.z = odom_.w.z();
    return msg;
  }

  void publish_odometry_outputs()
  {
    if (!have_odom_) {
      return;
    }

    const auto msg = make_odom_msg();
    if (publish_debug_odom_) {
      debug_odom_pub_->publish(msg);
    }
  }

  void log_diagnostics()
  {
    const auto throttle_ms = verbose_ ? 1000 : 5000;
    const auto current_time = now();
    const bool battery_ready =
      have_battery_ && (current_time - last_battery_time_).seconds() < msg_timeout_bat_;
    const double odom_age = have_odom_ ? (current_time - last_odom_time_).seconds() : -1.0;
    const double fcu_attitude_age =
      have_fcu_attitude_ ? (current_time - last_fcu_attitude_time_).seconds() : -1.0;
    const double setpoint_age = last_setpoint_publish_time_.nanoseconds() != 0 ?
      (current_time - last_setpoint_publish_time_).seconds() : -1.0;
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      throttle_ms,
      "[px4_ctrl_ros2] state=%s | px4=%s/%s | input: odom=%s status=%s rc=%s cmd=%s bat=%s | pos=(%.2f, %.2f, %.2f) err=%.2fm vel=%.2fm/s",
      state_name(state_),
      px4_mode_name(),
      arming_state_name(),
      ok_wait(odom_ready()),
      ok_wait(status_ready()),
      ok_wait(rc_control_available()),
      ok_wait(planner_cmd_ready()),
      ok_wait(battery_ready),
      odom_.p.x(),
      odom_.p.y(),
      odom_.p.z(),
      (odom_.p - hover_position_).norm(),
      odom_.v.norm());

    // RCLCPP_INFO_THROTTLE(
    //   get_logger(), *get_clock(), 1000,
    //   "[px4_ctrl_ros2] outputs: offboard_mode=%lu attitude_sp=%lu rates_sp=%lu "
    //   "subs(offboard=%zu attitude=%zu rates=%zu) "
    //   "age(setpoint=%.3fs fcu_att=%.3fs nav_odom=%.3fs) "
    //   "nav_odom(count=%lu last_gap=%.3fs max_gap=%.3fs)",
    //   static_cast<unsigned long>(offboard_control_mode_publish_count_),
    //   static_cast<unsigned long>(attitude_setpoint_publish_count_),
    //   static_cast<unsigned long>(rates_setpoint_publish_count_),
    //   offboard_control_mode_pub_->get_subscription_count(),
    //   attitude_setpoint_pub_->get_subscription_count(),
    //   rates_setpoint_pub_->get_subscription_count(),
    //   setpoint_age, fcu_attitude_age, odom_age,
    //   static_cast<unsigned long>(nav_odom_callback_count_),
    //   nav_odom_last_gap_, nav_odom_max_gap_);

    if (!verbose_) {
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "[px4_ctrl_ros2] detail: rc(mode=%.2f gear=%.2f hover=%s cmd=%s) | hover=(%.2f, %.2f, %.2f) stable=%s traj=%u/%u | ctrl(thr=%.2f thr2acc=%.2f acc_xy=%.2f pid_xy=%.2f vdes_xy=%.2f)",
      rc_.mode,
      rc_.gear,
      yes_no(rc_.is_hover_mode),
      yes_no(rc_.is_command_mode),
      hover_position_.x(),
      hover_position_.y(),
      hover_position_.z(),
      yes_no(hover_stable_started_),
      active_planner_traj_id_,
      completed_planner_traj_id_,
      controller_.debug().normalized_thrust,
      controller_.debug().thr2acc,
      controller_.debug().total_acc.head<2>().norm(),
      controller_.debug().pid_acc.head<2>().norm(),
      controller_.debug().desired_velocity.head<2>().norm());
  }

  double yaw_from_quaternion(const Eigen::Quaterniond &q) const
  {
    const Eigen::Vector3d x_axis = q * Eigen::Vector3d::UnitX();
    return std::atan2(x_axis.y(), x_axis.x());
  }

  rclcpp::Time now()
  {
    return get_clock()->now();
  }

  uint64_t timestamp_us()
  {
    return static_cast<uint64_t>(now().nanoseconds() / 1000);
  }

  const char *state_name(FlightState state) const
  {
    switch (state) {
      case FlightState::MANUAL_CTRL:
        return "MANUAL_CTRL";
      case FlightState::AUTO_HOVER:
        return "AUTO_HOVER";
      case FlightState::CMD_CTRL:
        return "CMD_CTRL";
      case FlightState::AUTO_TAKEOFF:
        return "AUTO_TAKEOFF";
      case FlightState::AUTO_LAND:
        return "AUTO_LAND";
      case FlightState::EXITING_OFFBOARD:
        return "EXITING_OFFBOARD";
      case FlightState::FAILSAFE:
        return "FAILSAFE";
    }
    return "UNKNOWN";
  }

  const char *px4_mode_name()
  {
    if (!status_ready()) {
      return "WAIT";
    }
    switch (vehicle_status_.nav_state) {
      case VehicleStatus::NAVIGATION_STATE_MANUAL:
        return "MANUAL";
      case VehicleStatus::NAVIGATION_STATE_ALTCTL:
        return "ALTCTL";
      case VehicleStatus::NAVIGATION_STATE_POSCTL:
        return "POSCTL";
      case VehicleStatus::NAVIGATION_STATE_ACRO:
        return "ACRO";
      case VehicleStatus::NAVIGATION_STATE_STAB:
        return "STAB";
      case VehicleStatus::NAVIGATION_STATE_OFFBOARD:
        return "OFFBOARD";
      default:
        return "OTHER";
    }
  }

  const char *arming_state_name()
  {
    if (!status_ready()) {
      return "WAIT";
    }
    return vehicle_is_armed() ? "ARMED" : "NOT_ARMED";
  }

  const char *ok_wait(bool value) const
  {
    return value ? "OK" : "WAIT";
  }

  const char *yes_no(bool value) const
  {
    return value ? "Y" : "N";
  }
};

}  // namespace px4_ctrl_ros2

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_ctrl_ros2::Px4CtrlNode>());
  rclcpp::shutdown();
  return 0;
}
