#include "px4_ctrl_ros2/controller.hpp"

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/input_rc.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
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

  enum class FlightState {
    MANUAL_CTRL,
    AUTO_HOVER,
    CMD_CTRL,
    AUTO_TAKEOFF,
    AUTO_LAND,
    FAILSAFE
  };

  ControlParams params_{};
  Controller controller_;
  RcState rc_{};
  OdomState odom_{};
  DesiredState planner_des_{};
  PositionCommand planner_cmd_{};
  VehicleStatus vehicle_status_{};
  VehicleLandDetected land_detected_{};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_pub_;
  rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr rates_setpoint_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr planner_trigger_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr planner_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr debug_odom_pub_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr nav_odom_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_fallback_sub_;
  rclcpp::Subscription<ManualControlSetpoint>::SharedPtr manual_control_sub_;
  rclcpp::Subscription<InputRc>::SharedPtr input_rc_sub_;
  rclcpp::Subscription<BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<VehicleLandDetected>::SharedPtr land_detected_sub_;
  rclcpp::Subscription<PositionCommand>::SharedPtr planner_cmd_sub_;
  rclcpp::Subscription<UInt8>::SharedPtr takeoff_land_sub_;

  FlightState state_{FlightState::MANUAL_CTRL};
  rclcpp::Time state_enter_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_rc_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_battery_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_land_detected_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_offboard_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_arm_request_time_{0, 0, RCL_ROS_TIME};

  bool have_odom_{false};
  bool have_status_{false};
  bool have_rc_{false};
  bool have_battery_{false};
  bool have_land_detected_{false};
  bool have_cmd_{false};
  bool planner_trigger_sent_for_command_{false};
  bool offboard_requested_{false};
  bool arm_requested_{false};
  bool hover_stable_started_{false};
  bool rc_required_{true};
  bool enable_offboard_command_{false};
  bool enable_auto_arm_{false};
  bool enable_auto_takeoff_land_{false};
  bool auto_start_planner_{false};
  bool publish_planner_odom_{true};
  bool publish_debug_odom_{true};
  bool estimate_nav_odom_velocity_{false};
  bool verbose_{false};
  bool reverse_roll_{false};
  bool reverse_pitch_{false};
  bool reverse_yaw_{false};
  bool reverse_throttle_{true};
  double msg_timeout_odom_{0.5};
  double msg_timeout_rc_{0.5};
  double msg_timeout_cmd_{0.5};
  double msg_timeout_bat_{0.5};
  double hover_stable_pos_tol_{0.30};
  double hover_stable_vel_tol_{0.30};
  double hover_stable_time_{1.0};
  double takeoff_height_{1.0};
  double takeoff_land_speed_{0.14};
  double battery_voltage_{14.0};
  uint64_t offboard_setpoint_counter_{0};
  uint32_t active_planner_traj_id_{0};
  uint32_t completed_planner_traj_id_{0};
  uint8_t takeoff_land_command_{0};
  Eigen::Vector3d hover_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_nav_odom_position_{Eigen::Vector3d::Zero()};
  double hover_yaw_{0.0};
  std::string odom_source_{"px4"};
  std::string odom_frame_id_{"world"};
  std::string odom_child_frame_id_{"base_link"};
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
    msg_timeout_rc_ = declare_parameter<double>("msg_timeout_rc", msg_timeout_rc_);
    msg_timeout_cmd_ = declare_parameter<double>("msg_timeout_cmd", msg_timeout_cmd_);
    msg_timeout_bat_ = declare_parameter<double>("msg_timeout_bat", msg_timeout_bat_);
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
    publish_planner_odom_ = declare_parameter<bool>("publish_planner_odom", publish_planner_odom_);
    hover_stable_pos_tol_ = declare_parameter<double>("hover_stable_pos_tol", hover_stable_pos_tol_);
    hover_stable_vel_tol_ = declare_parameter<double>("hover_stable_vel_tol", hover_stable_vel_tol_);
    hover_stable_time_ = declare_parameter<double>("hover_stable_time", hover_stable_time_);
    takeoff_height_ = declare_parameter<double>("takeoff_height", takeoff_height_);
    takeoff_land_speed_ = declare_parameter<double>("takeoff_land_speed", takeoff_land_speed_);
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
    planner_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("ego/odom_world", 10);
    debug_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("px4ctrl/debug_odom_enu", 10);

    if (odom_source_ == "nav") {
      nav_odom_sub_ = create_subscription<Odometry>(
        "nav/odom",
        rclcpp::QoS(20),
        std::bind(&Px4CtrlNode::nav_odometry_callback, this, std::placeholders::_1));
    } else {
      vehicle_odometry_sub_ = create_subscription<VehicleOdometry>(
        "px4/out/vehicle_odometry",
        px4_out_qos,
        std::bind(&Px4CtrlNode::vehicle_odometry_callback, this, std::placeholders::_1));
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
        handle_auto_takeoff_state(dt);
        break;
      case FlightState::AUTO_LAND:
        handle_auto_land_state(dt);
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
      if (rc_control_available() && (!rc_.is_hover_mode || !rc_.is_command_mode || !rc_.check_centered())) {
        RCLCPP_ERROR(
          get_logger(),
          "[px4_ctrl_ros2] Reject AUTO_TAKEOFF. If you have your RC connected, keep its switches at auto hover and command control states, and all sticks at the center, then takeoff again.");
        takeoff_land_command_ = 0;
        return;
      }

      set_hover_from_current(takeoff_height_);
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
      transition_to(FlightState::MANUAL_CTRL, "RC hover switch released or RC timeout");
      return;
    }

    if (takeoff_land_command_ == kLandCommand && enable_auto_takeoff_land_) {
      takeoff_land_command_ = 0;
      transition_to(FlightState::AUTO_LAND, "land command accepted");
      return;
    }

    update_hover_from_rc(dt);
    publish_control(make_hover_desired());

    const bool command_authorized = !rc_required_ || rc_.is_command_mode;
    const bool should_trigger_planner =
      (rc_required_ && rc_.is_command_mode) ||
      (!rc_required_ && auto_start_planner_);
    const bool hover_stable = hover_is_stable();
    if (command_authorized && should_trigger_planner && hover_stable && !planner_trigger_sent_for_command_) {
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
      transition_to(FlightState::MANUAL_CTRL, "RC hover switch released or RC timeout");
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

  void handle_auto_takeoff_state(double dt)
  {
    if (rc_required_ && !rc_control_available()) {
      transition_to(FlightState::MANUAL_CTRL, "RC timeout during takeoff");
      return;
    }

    hover_position_.z() = std::max(hover_position_.z(), odom_.p.z() + takeoff_land_speed_ * dt);
    publish_control(make_hover_desired());
    if (odom_.p.z() >= hover_position_.z() - 0.15) {
      transition_to(FlightState::AUTO_HOVER, "takeoff height reached");
    }
  }

  void handle_auto_land_state(double dt)
  {
    if (!odom_ready()) {
      transition_to(FlightState::FAILSAFE, "odometry lost during land");
      return;
    }

    hover_position_.z() = std::max(0.0, hover_position_.z() - takeoff_land_speed_ * dt);
    publish_control(make_hover_desired());

    if (have_land_detected_ && land_detected_.landed) {
      if (enable_offboard_command_ && enable_auto_arm_ && vehicle_is_armed()) {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
      }
      transition_to(FlightState::MANUAL_CTRL, "land detected");
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

  void publish_control(const DesiredState &des)
  {
    if (!enable_offboard_command_) {
      return;
    }

    publish_offboard_control_mode();
    const auto output = controller_.update_alg1(des, odom_, battery_voltage_);
    if (!control_output_is_finite(output)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[px4_ctrl_ros2] controller output is non-finite; skip setpoint");
      return;
    }
    const double thrust = std::clamp(output.thrust, 0.0, 1.0);

    if (params_.use_bodyrate_ctrl) {
      publish_rates_setpoint(output, thrust);
    } else {
      publish_attitude_setpoint(output, thrust, des.yaw_rate);
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
  }

  void publish_attitude_setpoint(const ControllerOutput &output, double thrust, double yaw_rate_enu)
  {
    const Eigen::Quaterniond q_ned_frd = enu_flu_to_ned_frd(output.q);
    VehicleAttitudeSetpoint msg{};
    msg.q_d = eigen_quat_to_px4_array(q_ned_frd);
    msg.thrust_body = {0.0f, 0.0f, static_cast<float>(-thrust)};
    msg.yaw_sp_move_rate = static_cast<float>(-yaw_rate_enu);
    msg.reset_integral = false;
    msg.fw_control_yaw_wheel = false;
    msg.timestamp = timestamp_us();
    attitude_setpoint_pub_->publish(msg);
  }

  void publish_rates_setpoint(const ControllerOutput &output, double thrust)
  {
    VehicleRatesSetpoint msg{};
    msg.roll = static_cast<float>(output.bodyrates.x());
    msg.pitch = static_cast<float>(-output.bodyrates.y());
    msg.yaw = static_cast<float>(-output.bodyrates.z());
    msg.thrust_body = {0.0f, 0.0f, static_cast<float>(-thrust)};
    msg.reset_integral = false;
    msg.timestamp = timestamp_us();
    rates_setpoint_pub_->publish(msg);
  }

  void maybe_request_offboard_and_arm()
  {
    if (offboard_setpoint_counter_ < 10) {
      return;
    }

    const auto current_time = now();
    if (!vehicle_is_offboard() &&
      (last_offboard_request_time_.nanoseconds() == 0 ||
      (current_time - last_offboard_request_time_).seconds() > 1.0)) {
      publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
      offboard_requested_ = true;
      last_offboard_request_time_ = current_time;
      RCLCPP_INFO(get_logger(), "[px4_ctrl_ros2] OFFBOARD mode command sent");
    }

    if (enable_auto_arm_ && !vehicle_is_armed() &&
      (last_arm_request_time_.nanoseconds() == 0 ||
      (current_time - last_arm_request_time_).seconds() > 1.0)) {
      publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
      arm_requested_ = true;
      last_arm_request_time_ = current_time;
      RCLCPP_INFO(get_logger(), "[px4_ctrl_ros2] ARM command sent");
    }
  }

  void publish_vehicle_command(uint32_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
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
    hover_stable_started_ = false;
    active_planner_traj_id_ = 0;
    controller_.reset_thrust_mapping();
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
    arm_requested_ = false;
    last_offboard_request_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_arm_request_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    planner_trigger_sent_for_command_ = false;
    hover_stable_started_ = false;
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
    return have_status_ && (now() - last_status_time_).seconds() < msg_timeout_odom_;
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
    if (msg->pose_frame != VehicleOdometry::POSE_FRAME_NED ||
      msg->velocity_frame != VehicleOdometry::VELOCITY_FRAME_NED) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "[px4_ctrl_ros2] vehicle_odometry frame is not NED/NED; pose_frame=%u velocity_frame=%u",
        static_cast<unsigned>(msg->pose_frame),
        static_cast<unsigned>(msg->velocity_frame));
    }

    const Eigen::Vector3d p_ned(msg->position[0], msg->position[1], msg->position[2]);
    const Eigen::Vector3d v_ned(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    const Eigen::Quaterniond q_ned_frd(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    if (!finite_vector3(p_ned) || !finite_vector3(v_ned) ||
      !std::isfinite(q_ned_frd.w()) || !std::isfinite(q_ned_frd.x()) ||
      !std::isfinite(q_ned_frd.y()) || !std::isfinite(q_ned_frd.z())) {
      return;
    }

    odom_.p = ned_to_enu(p_ned);
    odom_.v = ned_to_enu(v_ned);
    odom_.q = ned_frd_to_enu_flu(q_ned_frd.normalized());
    odom_.w = Eigen::Vector3d(
      msg->angular_velocity[0],
      -msg->angular_velocity[1],
      -msg->angular_velocity[2]);
    have_odom_ = true;
    last_odom_time_ = now();
    odom_frame_id_ = "world";
    odom_child_frame_id_ = "base_link";
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
    last_odom_time_ = now();
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
    if (publish_planner_odom_) {
      planner_odom_pub_->publish(msg);
    }
    if (publish_debug_odom_) {
      debug_odom_pub_->publish(msg);
    }
  }

  void log_diagnostics()
  {
    const auto throttle_ms = verbose_ ? 1000 : 5000;
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      throttle_ms,
      "[px4_ctrl_ros2] state=%s output=%s odom_source=%s | px4(nav=%u arm=%u offboard=%s armed=%s) | odom=%s status=%s rc=%s cmd=%s bat=%s | rc(mode=%.2f gear=%.2f hover=%s cmd=%s) | pos=%.2f %.2f %.2f hover=%.2f %.2f %.2f err=%.2f vel=%.2f stable=%s traj=%u/%u | ctrl(thr=%.2f acc_xy=%.2f pid_xy=%.2f vdes_xy=%.2f)",
      state_name(state_),
      yes_no(enable_offboard_command_),
      odom_source_.c_str(),
      have_status_ ? static_cast<unsigned>(vehicle_status_.nav_state) : 255U,
      have_status_ ? static_cast<unsigned>(vehicle_status_.arming_state) : 255U,
      yes_no(vehicle_is_offboard()),
      yes_no(vehicle_is_armed()),
      ok_wait(odom_ready()),
      ok_wait(status_ready()),
      ok_wait(rc_control_available()),
      ok_wait(planner_cmd_ready()),
      ok_wait(have_battery_ && (now() - last_battery_time_).seconds() < msg_timeout_bat_),
      rc_.mode,
      rc_.gear,
      yes_no(rc_.is_hover_mode),
      yes_no(rc_.is_command_mode),
      odom_.p.x(),
      odom_.p.y(),
      odom_.p.z(),
      hover_position_.x(),
      hover_position_.y(),
      hover_position_.z(),
      (odom_.p - hover_position_).norm(),
      odom_.v.norm(),
      yes_no(hover_stable_started_),
      active_planner_traj_id_,
      completed_planner_traj_id_,
      controller_.debug().normalized_thrust,
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
      case FlightState::FAILSAFE:
        return "FAILSAFE";
    }
    return "UNKNOWN";
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
