# EPFL AI Team - SAPIEN

# Author: Mattia Prandi, 2026

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    my_params = os.path.join(get_package_share_directory('mina_navigation'), 'param', 'nav2_params.yaml')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'params_file': my_params,
                'use_sim_time': 'true' # Setta a false se sei sul robot reale
            }.items()
        ),
    ])