import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess,SetLaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import yaml
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_dir = get_package_share_directory('groundstation_launch')
    config_file = os.path.join(pkg_dir, 'config/simulation', 'gazebo.param.yaml')#groundstation_launch/config/simulation/gazebo.param.yaml
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    n_agent_value = config.get('/**', {}).get('ros__parameters', {}).get('num_agents', 2)
    SetLaunchConfiguration('n_agent', str(n_agent_value))
    # Set the n_agent value in the launch configuration

    # Correct path with the fixed spelling
    micro_xrec_agent = ExecuteProcess(
        cmd=[[
            'MicroXRCEAgent udp4 --port 8888 --addr 127.0.0.1 -d 100'
        ]],
        shell=True
    )
    px4_process = ExecuteProcess(
        cmd=[
            "~/uav_universe/src/groundstation_setting/groundstation_launch/script/run_gzebo_Micro.sh " + str(n_agent_value)
            # "~/Projects/NTU/Lab/ros2_ws/src/groundstation_setting/groundstation_launch/script/run_gzebo_Micro.sh "+ str(n_agent_value) 
        ],
        shell=True
    )
    arming_takeoff_setpoints_process = ExecuteProcess(
        cmd=[
            "ros2 launch arming_takeoff_setpoints arming_takeoff_setpoints.launch.py"
        ],
        shell=True
    )

    return LaunchDescription([
        micro_xrec_agent,
        px4_process,
        arming_takeoff_setpoints_process])
