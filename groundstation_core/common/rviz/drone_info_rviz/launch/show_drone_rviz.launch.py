from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build_nodes(context, *args, **kwargs):
    mode       = LaunchConfiguration("mode").perform(context)
    log_level  = LaunchConfiguration("log_level").perform(context)
    rviz_cfg   = LaunchConfiguration("rviz_config").perform(context)

    # ───────────── topic presets per mode ─────────────
    if mode == "sim":
        topic_name_traj_setpoint_ref_101 = "/rviz/drone101/send_setpoint_xyzyaw/enu"
        topic_name_odom_101              = "/px4_1/fmu/out/vehicle_odometry_unbiase_z/enu"

        topic_name_traj_setpoint_ref_102 = "/rviz/drone101/send_setpoint_xyzyaw/enu"
        topic_name_odom_102              = "/px4_2/fmu/out/vehicle_odometry_unbiase_z/enu"

        topic_name_traj_setpoint_ref_103 = "/rviz/drone101/send_setpoint_xyzyaw/enu"
        topic_name_odom_103              = "/px4_2/fmu/out/vehicle_odometry_unbiase_z/enu"
    else:  # real hardware
        topic_name_traj_setpoint_ref_101 = "/rviz/drone101/send_setpoint_xyzyaw/enu"
        topic_name_odom_101              = "/stationui_backend/drone101/recv_pose/enu"

        topic_name_traj_setpoint_ref_102 = "/rviz/drone102/send_setpoint_xyzyaw/enu"
        topic_name_odom_102              = "/stationui_backend/drone102/recv_pose/enu"

        topic_name_traj_setpoint_ref_103 = "/rviz/drone103/send_setpoint_xyzyaw/enu"
        topic_name_odom_103              = "/stationui_backend/drone103/recv_pose/enu"

    drone_rviz = Node(
        package="drone_info_rviz",
        executable="drone_info_rviz",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("px4_1/setpoint_ref", topic_name_traj_setpoint_ref_101),
            ("px4_1/odom",        topic_name_odom_101),
            ("px4_2/setpoint_ref", topic_name_traj_setpoint_ref_102),
            ("px4_2/odom",        topic_name_odom_102),
            ("px4_3/setpoint_ref", topic_name_traj_setpoint_ref_103),
            ("px4_3/odom",        topic_name_odom_103),
        ],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
    )

    return [drone_rviz, rviz2]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode", default_value="sim",
            description="Operating mode: real | sim"
        ),
        DeclareLaunchArgument(
            "log_level", default_value="info",
            description="Log severity: debug | info | warn | error | fatal"
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("groundstation_core"),
                 "common", "rviz", "config", "uav_universe.rviz"]),
            description="RViz2 configuration file"
        ),
        OpaqueFunction(function=_build_nodes)
    ])
