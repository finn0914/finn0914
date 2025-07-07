from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml

def generate_launch_description():
    # Get the directory of your package
    pkg_dir = get_package_share_directory('groundstation_launch')
    config_file = os.path.join(pkg_dir, 'config/simulation', 'gazebo.param.yaml')#groundstation_launch/config/simulation/gazebo.param.yaml
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    n_agent_value = config.get('/**', {}).get('ros__parameters', {}).get('num_agents', 1)

    pkg_dir = get_package_share_directory('keep_distance_pid')
    config_file = os.path.join(pkg_dir, 'config', 'controller_param.param.yaml')#groundstation_launch/config/simulation/gazebo.param.yaml
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    controller_name = config.get('/**', {}).get('ros__parameters', {}).get('controller_name', 'pid')
    distance = config.get('/**', {}).get('ros__parameters', {}).get('distance', 1)

    kpx = config.get('/**', {}).get('ros__parameters', {}).get('kpx', 1)
    kix = config.get('/**', {}).get('ros__parameters', {}).get('kix', 1)
    kdx = config.get('/**', {}).get('ros__parameters', {}).get('kdx', 1)

    kpy = config.get('/**', {}).get('ros__parameters', {}).get('kpy', 1)
    kiy = config.get('/**', {}).get('ros__parameters', {}).get('kiy', 1)
    kdy = config.get('/**', {}).get('ros__parameters', {}).get('kdy', 1)
    
    kpz = config.get('/**', {}).get('ros__parameters', {}).get('kpz', 1)
    kiz = config.get('/**', {}).get('ros__parameters', {}).get('kiz', 1)
    kdz = config.get('/**', {}).get('ros__parameters', {}).get('kdz', 1)
    
    log_level_var=DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (info, warn, debug, error)'
    )
    log_level = LaunchConfiguration('log_level')
    node_1 = Node(
        package='keep_distance_pid',
        executable='keep_distance_pid',
        name='keep_distance_pid_core',
        parameters=[
            {"distance":distance},
            {'num_agents': n_agent_value},
            {'controller_name' : controller_name},
            # x channel
            {'kpx': kpx},
            {'kix': kix},
            {'kdx': kdx},
            # y channel
            {'kpy': kpy},
            {'kiy': kiy},
            {'kdy': kdy},
             # z channel
            {'kpz': kpz},
            {'kiz': kiz},
            {'kdz': kdz}
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', log_level]  # Pass log_level to node
    )


    return LaunchDescription([log_level_var,node_1])