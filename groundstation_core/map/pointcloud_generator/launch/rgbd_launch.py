from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='rgbd_to_pointcloud',
            name='pointcloud_node',
            output='screen'
        )
    ])
