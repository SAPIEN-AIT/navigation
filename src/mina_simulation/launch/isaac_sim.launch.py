import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import logging

def generate_launch_description():
    logging.getLogger('launch').setLevel(logging.ERROR)
    # 1. Configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Isaac) clock if true'
    )

    # Define the path to your Isaac Sim installation
    default_isaac_sim_path = os.path.expanduser('/isaac-sim/isaac-sim.sh')
    
    isaac_sim_path_arg = DeclareLaunchArgument(
        'isaac_sim_path',
        default_value=default_isaac_sim_path,
        description='Absolute path to the isaac-sim.sh executable'
    )

    isaac_sim_path = LaunchConfiguration('isaac_sim_path')

    # 2. Path to the USD world file
    pkg_share = get_package_share_directory('mina_simulation')
    usd_file_path = os.path.join(pkg_share, 'worlds', 'environment.usd')

    # 3. Process to launch the external Isaac Sim application
    isaac_sim_process = ExecuteProcess(
        cmd=[isaac_sim_path, usd_file_path],
        # cmd=[isaac_sim_path, usd_file_path, '--no-window'],
        output='log'
    )

    return LaunchDescription([
        use_sim_time_arg,
        isaac_sim_path_arg,
        isaac_sim_process
    ])