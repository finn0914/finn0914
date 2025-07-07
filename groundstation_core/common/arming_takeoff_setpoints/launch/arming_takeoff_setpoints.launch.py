from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
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

    pkg_dir = get_package_share_directory('arming_takeoff_setpoints')
    config_file = os.path.join(pkg_dir, 'config', 'agents.param.yaml')#groundstation_launch/config/simulation/gazebo.param.yaml
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    sphare_radius = config.get('/**', {}).get('ros__parameters', {}).get('sphare_radius', 1)


    log_level_var=DeclareLaunchArgument(
        'log_level',
        default_value='debug',
        description='Logging level (info, warn, debug, error)'
    )
    log_level = LaunchConfiguration('log_level')
    # map_param_config_path = LaunchConfiguration('map_param_config_path')
    multi_container = ComposableNodeContainer(
            name='multi_agent_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',  # Multithreaded container
            composable_node_descriptions=[
                # ComposableNode(
                #     package='arming_takeoff_setpoints',# its important which plugin starts first!so ,server first
                #     plugin='FollowTraj',
                #     name='follow_traj_core',
                #     extra_arguments=[{'use_intra_process_comms': True}],

                #     parameters=[{'num_agents': n_agent_value}]

                # ),
                ComposableNode(
                    package='arming_takeoff_setpoints',
                    plugin='LeaderFollower',
                    name='leader_follower_core',
                    extra_arguments=[{'use_intra_process_comms': True}],

                    parameters=[{'num_agents': n_agent_value},{'sphare_radius': sphare_radius}]

                ),
                ComposableNode(
                    package='arming_takeoff_setpoints',
                    plugin='ARMDISARM',# class name
                    name='arm_disarm_core', # node name
                    extra_arguments=[{'use_intra_process_comms': True}], # this line avoid serialization and deseralization .

                    parameters=[{'num_agents': n_agent_value}]
                ),
                ComposableNode(
                    package='arming_takeoff_setpoints',
                    plugin='TakeOff',
                    name='take_off_core',
                    extra_arguments=[{'use_intra_process_comms': True}],

                    parameters=[{'num_agents': n_agent_value}]

                ),

            ],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level]  # Pass log_level to container

        )
    return LaunchDescription([log_level_var,multi_container])