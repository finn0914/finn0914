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

    pkg_dir = get_package_share_directory('generate_trajectory')
    config_file = os.path.join(pkg_dir, 'config', 'gen_traj.param.yaml')#generate_trajectory/config/gen_traj.param.yaml
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    # setpoint_delay = config.get('/**', {}).get('ros__parameters', {}).get('setpoint_delay', 1)
    # circle_res = config.get('/**', {}).get('ros__parameters', {}).get('circle_resolution', 1)
    # only_line_x = config.get('/**', {}).get('ros__parameters', {}).get('only_line_x', 1)
    # only_line_y = config.get('/**', {}).get('ros__parameters', {}).get('only_line_y', 1)
    # only_line_z = config.get('/**', {}).get('ros__parameters', {}).get('only_line_z', 1)


    log_level_var=DeclareLaunchArgument(
        'log_level',
        default_value='debug',
        description='Logging level (info, warn, debug, error)'
    )
    log_level = LaunchConfiguration('log_level')
    # print("=== Parse-time debug prints ===")
    # print(f"n_agent_value: {n_agent_value}")
    # print(f"setpoint_delay: {setpoint_delay}")
    # print(f"circle_resolution: {circle_res}")
    # print(f"log_level (LaunchConfiguration): {log_level}")  
    # map_param_config_path = LaunchConfiguration('map_param_config_path')
    multi_container = ComposableNodeContainer(
            name='multi_container_planner',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',  # Multithreaded container
            composable_node_descriptions=[
                ComposableNode(
                    package='generate_trajectory',
                    plugin='PlanningForLeader',
                    name='planning_for_leader_core',
                    extra_arguments=[{'use_intra_process_comms': True}],
                    parameters=[config_file]
                ),
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level]  # Pass log_level to container

        )
    return LaunchDescription([log_level_var,multi_container])