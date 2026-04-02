"""
RTABMap mapping + Nav2 navigation with ZED2i.

Launches:
  1. zed2i_rtabmap_mapping (existing launch — ZED + RTABMap SLAM)
  2. depthimage_to_laserscan (converts ZED depth → /scan for Nav2 costmaps)
  3. Nav2 bringup (planner, controller, costmaps, behaviors)

Usage:
  ros2 launch mina_navigation rtabmap_nav2.launch.py
  ros2 launch mina_navigation rtabmap_nav2.launch.py serial_number:=35134495
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    camera_name = LaunchConfiguration('camera_name').perform(context)
    zed_node_name = LaunchConfiguration('zed_node_name').perform(context)

    zed_topic_root = f'/{camera_name}/{zed_node_name}'

    actions = []

    # ── 1. RTABMap mapping (existing launch) ──
    rtabmap_pkg = get_package_share_directory('zed2i_rtabmap')
    rtabmap_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_pkg, 'launch', 'zed2i_rtabmap_mapping.launch.py')
        ),
        launch_arguments={
            'camera_name': camera_name,
            'serial_number': LaunchConfiguration('serial_number').perform(context),
            'zed_node_name': zed_node_name,
            'use_rviz': 'false',  # Nav2 has its own RViz config
            'use_rtabmap_viz': 'false',
        }.items(),
    )
    actions.append(rtabmap_mapping)

    # ── 2. Depth image → LaserScan ──
    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'scan_height': 10,           # Rows of depth image to use
            'scan_time': 0.033,          # ~30 Hz
            'range_min': 0.3,            # Match ZED/RTABMap min range
            'range_max': 8.0,            # Match ZED/RTABMap max range
            'output_frame_id': f'{camera_name}_left_camera_frame',
        }],
        remappings=[
            ('depth', f'{zed_topic_root}/depth/depth_registered'),
            ('depth_camera_info', f'{zed_topic_root}/depth/camera_info'),
            ('scan', '/scan'),
        ],
    )
    actions.append(depth_to_scan)

    # ── 3. Nav2 bringup ──
    nav2_params = LaunchConfiguration('nav2_params').perform(context)

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py',  # Navigation only (no AMCL/map_server — RTABMap handles that)
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'autostart': 'true',
        }.items(),
    )
    actions.append(nav2_bringup)

    # ── 4. RViz ──
    rviz_config = os.path.join(
        get_local_share_dir(), 'config', 'rtabmap_nav2.rviz'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen',
    )
    actions.append(rviz)

    return actions


def get_local_share_dir():
    try:
        return get_package_share_directory('mina_navigation')
    except PackageNotFoundError:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    pkg_dir = get_local_share_dir()

    default_nav2_params = os.path.join(pkg_dir, 'param', 'nav2_rtabmap_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='zed2i'),
        DeclareLaunchArgument('serial_number', default_value='35134495'),
        DeclareLaunchArgument('zed_node_name', default_value='zed_node'),
        DeclareLaunchArgument('nav2_params', default_value=default_nav2_params,
                              description='Path to Nav2 parameters file'),
        OpaqueFunction(function=launch_setup),
    ])
