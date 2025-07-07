#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description():
    # 1) Declare all launch arguments BEFORE using them
    run_mode_arg = DeclareLaunchArgument(
        'run_mode',
        default_value='sim',
        description='sim/real'
    )
    log_level_var = DeclareLaunchArgument(
        'log_level',
        default_value='debug',
        description='Logging level (info, warn, debug, error)'
    )
    declare_drone101_arg = DeclareLaunchArgument(
        'drone101',
        default_value='True',
        description='Enable static TFs for drone101'
    )
    declare_drone102_arg = DeclareLaunchArgument(
        'drone102',
        default_value='True',
        description='Enable static TFs for drone102'
    )
    declare_drone103_arg = DeclareLaunchArgument(
        'drone103',
        default_value='True',
        description='Enable static TFs for drone103'
    )
    # mode       = LaunchConfiguration("mode")
    drone101_flag = LaunchConfiguration('drone101')
    drone102_flag = LaunchConfiguration('drone102')
    drone103_flag = LaunchConfiguration('drone103')

    pkg_dir = get_package_share_directory('groundstation_launch')
    config_file = os.path.join(pkg_dir, 'config/simulation', 'gazebo.param.yaml')#groundstation_launch/config/simulation/gazebo.param.yaml
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    n_agent_value = config.get('/**', {}).get('ros__parameters', {}).get('num_agents', 1)
    
    topics_conversion = Node(
        package='tf_tree', executable='topics_conversion_main',
        name='topics_conversion_main',
        parameters=[{'num_agents': n_agent_value}],

        output='screen', condition=IfCondition(drone101_flag)
    )

    map_enu_static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='station_ned_to_world_enu_tf',
        arguments=[ '0.00','0.00','0.00','0.70710678','0.70710678','0','0','map_enu','station_ned'],
        output='screen', condition=IfCondition(drone101_flag)
    )
    # Drone101 static TFs
    # station_ned_static_tf_d1 = Node(
    #     package='tf2_ros', executable='static_transform_publisher',
    #     name='drone101_base_link_to_station_ned_tf',
    #     arguments=[ '0.00','0.00','0.00','0','0','0','1','station_ned','drone101/base_link'],
    #     output='screen', condition=IfCondition(drone101_flag)
    # )
    imu_static_tf_d1 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='drone101_imu_static_tf',
        arguments=[ '0.00','0.00','0.00','0','0','0','1','drone101/base_link','drone101/imu_link'],
        output='screen', condition=IfCondition(drone101_flag)
    )
    ultrasonic_static_tf_d1 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='drone101_ultrasonic_static_tf',
        arguments=[ '0.00','0.00','-0.10','0','0','0','1','drone101/base_link','drone101/ultrasonic_link'],
        output='screen', condition=IfCondition(drone101_flag)
    )
    camera_static_tf_d1 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='drone101_camera_static_tf',
        arguments=[ '0.10','0.00','0.00','0','0','0','1','drone101/base_link','drone101/camera_link'],
        output='screen', condition=IfCondition(drone101_flag)
    )
    linktrack_static_tf_d1 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='drone101_linktrack_static_tf',
        arguments=[ '0.00','-0.10','0.00','0','0','0','1','drone101/base_link','drone101/linktrack_link'],
        output='screen', condition=IfCondition(drone101_flag)
    )

    # Drone102 static TFs
    # station_ned_static_tf_d2 = Node(
    #     package='tf2_ros', executable='static_transform_publisher',
    #     name='drone102_base_link_to_station_ned_tf',
    #     arguments=[ '0.00','0.00','0.00','0','0','0','1','station_ned','drone102/base_link'],
    #     output='screen', condition=IfCondition(drone102_flag)
    # )
    imu_static_tf_d2 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='drone102_imu_static_tf',
        arguments=[ '0.00','0.00','0.00','0','0','0','1','drone102/base_link','drone102/imu_link'],
        output='screen', condition=IfCondition(drone102_flag)
    )
    ultrasonic_static_tf_d2 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='drone102_ultrasonic_static_tf',
        arguments=[ '0.00','0.00','-0.10','0','0','0','1','drone102/base_link','drone102/ultrasonic_link'],
        output='screen', condition=IfCondition(drone102_flag)
    )
    camera_static_tf_d2 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='drone102_camera_static_tf',
        arguments=[ '0.10','0.00','0.00','0','0','0','1','drone102/base_link','drone102/camera_link'],
        output='screen', condition=IfCondition(drone102_flag)
    )
    linktrack_static_tf_d2 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='drone102_linktrack_static_tf',
        arguments=[ '0.00','-0.10','0.00','0','0','0','1','drone102/base_link','drone102/linktrack_link'],
        output='screen', condition=IfCondition(drone102_flag)
    )

    # Drone103 static TFs
    #station_ned_static_tf_d3 = Node(
    #     package='tf2_ros', executable='static_transform_publisher',
    #     name='drone103_base_link_to_station_ned_tf',
    #     arguments=[ '0.00','0.00','0.00','0','0','0','1','station_ned','drone103/base_link'],
    #     output='screen', condition=IfCondition(drone103_flag)
    # )
    imu_static_tf_d3 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='drone103_imu_static_tf',
        arguments=[ '0.00','0.00','0.00','0','0','0','1','drone103/base_link','drone103/imu_link'],
        output='screen', condition=IfCondition(drone103_flag)
    )
    ultrasonic_static_tf_d3 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='drone103_ultrasonic_static_tf',
        arguments=[ '0.00','0.00','-0.10','0','0','0','1','drone103/base_link','drone103/ultrasonic_link'],
        output='screen', condition=IfCondition(drone103_flag)
    )
    camera_static_tf_d3 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='drone103_camera_static_tf',
        arguments=[ '0.10','0.00','0.00','0','0','0','1','drone103/base_link','drone103/camera_link'],
        output='screen', condition=IfCondition(drone103_flag)
    )
    linktrack_static_tf_d3 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='drone103_linktrack_static_tf',
        arguments=[ '0.00','-0.10','0.00','0','0','0','1','drone103/base_link','drone103/linktrack_link'],
        output='screen', condition=IfCondition(drone103_flag)
    )

    # 5) Assemble the launch description in correct order
    ld = LaunchDescription([
        run_mode_arg,
        log_level_var,
        declare_drone101_arg,
        declare_drone102_arg,
        declare_drone103_arg,

        topics_conversion,
        map_enu_static_tf,

        #station_ned_static_tf_d1,
        imu_static_tf_d1,
        ultrasonic_static_tf_d1,
        camera_static_tf_d1,
        linktrack_static_tf_d1,

        #station_ned_static_tf_d2,
        imu_static_tf_d2,
        ultrasonic_static_tf_d2,
        camera_static_tf_d2,
        linktrack_static_tf_d2,

        #station_ned_static_tf_d3,
        imu_static_tf_d3,
        ultrasonic_static_tf_d3,
        camera_static_tf_d3,
        linktrack_static_tf_d3
    ])

    return ld
