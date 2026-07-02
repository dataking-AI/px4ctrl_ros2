import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use ROS simulation time. Keep false unless /clock is available.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                get_package_share_directory("px4_ctrl_ros2"),
                "config",
                "ctrl_param_fpv.yaml",
            ),
            description="ROS2 YAML parameters for px4_ctrl_node.",
        ),
        DeclareLaunchArgument(
            "offboard_control_mode_topic",
            default_value="/fmu/in/offboard_control_mode",
            description="PX4 offboard control mode input topic.",
        ),
        DeclareLaunchArgument(
            "vehicle_command_topic",
            default_value="/fmu/in/vehicle_command",
            description="PX4 vehicle command input topic.",
        ),
        DeclareLaunchArgument(
            "vehicle_attitude_setpoint_topic",
            default_value="/fmu/in/vehicle_attitude_setpoint",
            description="PX4 attitude setpoint input topic.",
        ),
        DeclareLaunchArgument(
            "vehicle_rates_setpoint_topic",
            default_value="/fmu/in/vehicle_rates_setpoint",
            description="PX4 rates setpoint input topic.",
        ),
        DeclareLaunchArgument(
            "vehicle_odometry_topic",
            default_value="/fmu/out/vehicle_odometry",
            description="PX4 vehicle odometry output topic.",
        ),
        DeclareLaunchArgument(
            "vehicle_status_topic",
            default_value="/fmu/out/vehicle_status_v1",
            description="PX4 vehicle status output topic used by newer PX4 message versions.",
        ),
        DeclareLaunchArgument(
            "vehicle_status_fallback_topic",
            default_value="/fmu/out/vehicle_status",
            description="PX4 vehicle status output topic used by older PX4 message versions.",
        ),
        DeclareLaunchArgument(
            "manual_control_topic",
            default_value="/fmu/out/manual_control_setpoint",
            description="PX4 normalized manual control output topic.",
        ),
        DeclareLaunchArgument(
            "input_rc_topic",
            default_value="/fmu/out/input_rc",
            description="PX4 raw RC input output topic.",
        ),
        DeclareLaunchArgument(
            "battery_status_topic",
            default_value="/fmu/out/battery_status",
            description="PX4 battery status output topic.",
        ),
        DeclareLaunchArgument(
            "vehicle_land_detected_topic",
            default_value="/fmu/out/vehicle_land_detected",
            description="PX4 land detector output topic.",
        ),
        DeclareLaunchArgument(
            "planner_pos_cmd_topic",
            default_value="/drone_0_planning/pos_cmd",
            description="EGO traj_server PositionCommand output topic.",
        ),
        DeclareLaunchArgument(
            "planner_trigger_topic",
            default_value="/traj_start_trigger",
            description="EGO PRESET_TARGET start trigger topic.",
        ),
        DeclareLaunchArgument(
            "takeoff_land_topic",
            default_value="/px4ctrl/takeoff_land_cmd",
            description="UInt8 takeoff/land command topic: 1=takeoff, 2=land.",
        ),
        DeclareLaunchArgument(
            "debug_odom_topic",
            default_value="/px4ctrl/debug_odom_enu",
            description="Debug ENU odometry converted from PX4 odometry.",
        ),
        Node(
            package="px4_ctrl_ros2",
            executable="px4_ctrl_node",
            name="px4_ctrl_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                params_file,
                {"use_sim_time": use_sim_time},
            ],
            remappings=[
                ("px4/in/offboard_control_mode", LaunchConfiguration("offboard_control_mode_topic")),
                ("px4/in/vehicle_command", LaunchConfiguration("vehicle_command_topic")),
                ("px4/in/vehicle_attitude_setpoint", LaunchConfiguration("vehicle_attitude_setpoint_topic")),
                ("px4/in/vehicle_rates_setpoint", LaunchConfiguration("vehicle_rates_setpoint_topic")),
                ("px4/out/vehicle_odometry", LaunchConfiguration("vehicle_odometry_topic")),
                ("px4/out/vehicle_status_v1", LaunchConfiguration("vehicle_status_topic")),
                ("px4/out/vehicle_status", LaunchConfiguration("vehicle_status_fallback_topic")),
                ("px4/out/manual_control_setpoint", LaunchConfiguration("manual_control_topic")),
                ("px4/out/input_rc", LaunchConfiguration("input_rc_topic")),
                ("px4/out/battery_status", LaunchConfiguration("battery_status_topic")),
                ("px4/out/vehicle_land_detected", LaunchConfiguration("vehicle_land_detected_topic")),
                ("ego/position_cmd", LaunchConfiguration("planner_pos_cmd_topic")),
                ("ego/traj_start_trigger", LaunchConfiguration("planner_trigger_topic")),
                ("px4ctrl/takeoff_land_cmd", LaunchConfiguration("takeoff_land_topic")),
                ("px4ctrl/debug_odom_enu", LaunchConfiguration("debug_odom_topic")),
            ],
        ),
    ])
