from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare 'use_sim_time' as a launch argument, default to 'false'
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    return LaunchDescription([
        # Add the declared launch argument to the launch description
        use_sim_time_arg,

        # Node for pose_to_tf_node
        Node(
            package='station_flight',  # Your package name
            executable='pose_to_tf_node',  # Your executable name
            name='pose_to_tf_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],  # Adding use_sim_time parameter
            arguments=['--ros-args', '--log-level', 'info'],  # Setting log level to info
        ),

        # Static transform publisher for drone 103
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_Dr103_camera',
            arguments=['0.25', '0.0', '-0.15', '-1.57', '0', '-1.3', '/drone103/base_link', '/drone103/camera_depth_tf']
        ),

        # Static transform publisher for drone 102
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_Dr102_camera',
            arguments=['0.25', '0.0', '-0.15', '-1.57', '0', '-1.3', '/drone102/base_link', '/drone102/camera_depth_tf']
        ),

        # Static transform publisher for drone 101
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_Dr101_camera',
            arguments=['0.25', '0.0', '-0.15', '-1.57', '0', '-1.3', '/drone101/base_link', '/drone101/camera_depth_tf']
        ),
    ])
