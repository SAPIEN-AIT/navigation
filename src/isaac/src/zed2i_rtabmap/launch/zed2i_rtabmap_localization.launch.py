"""
RTABMap localization + Nav2 navigation with a previously saved map.

This launch file starts:
  1. ZED2i camera wrapper (with odometry)
  2. RTABMap in LOCALIZATION mode (reuses the database from mapping)
  3. Nav2 bringup (navigation stack)

Usage:
  ros2 launch zed2i_rtabmap zed2i_rtabmap_localization.launch.py \
      map:=/path/to/my_map.yaml

Prerequisites:
  - A 2D map saved during the mapping phase (map.yaml + map.pgm)
  - RTABMap database from mapping session (default: /tmp/rtabmap_zed2i.db)
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_local_share_dir():
    try:
        return get_package_share_directory('zed2i_rtabmap')
    except PackageNotFoundError:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def launch_setup(context, *args, **kwargs):
    camera_name = LaunchConfiguration('camera_name').perform(context)
    zed_node_name = LaunchConfiguration('zed_node_name').perform(context)
    launch_zed = LaunchConfiguration('launch_zed_wrapper').perform(context).lower() in ('1', 'true', 'yes')

    pkg_dir = get_local_share_dir()
    rtabmap_params = os.path.join(pkg_dir, 'config', 'rtabmap_params.yaml')
    zed_camera_config = os.path.join(pkg_dir, 'config', 'zed2i_camera.yaml')

    zed_topic_root = f'/{camera_name}/{zed_node_name}'

    actions = []

    # ── 1. ZED camera wrapper ──
    if launch_zed:
        zed_wrapper_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('zed_wrapper'),
                    'launch',
                    'zed_camera.launch.py',
                )
            ),
            launch_arguments={
                'camera_name': camera_name,
                'camera_model': 'zed2i',
                'serial_number': LaunchConfiguration('serial_number').perform(context),
                'ros_params_override_path': zed_camera_config,
                'publish_urdf': 'true',
                'publish_tf': 'true',
                'publish_map_tf': 'false',
                'publish_imu_tf': 'true',
            }.items(),
        )
        actions.append(zed_wrapper_launch)

    # ── 2. RTABMap in LOCALIZATION mode ──
    rtabmap_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_params,
            {
                'Mem/IncrementalMemory': 'false',       # Localization mode
                'Mem/InitWMWithAllNodes': 'true',        # Load all nodes for localization
            },
        ],
        remappings=[
            ('rgb/image', f'{zed_topic_root}/rgb/image_rect_color'),
            ('depth/image', f'{zed_topic_root}/depth/depth_registered'),
            ('rgb/camera_info', f'{zed_topic_root}/rgb/camera_info'),
            ('odom', f'{zed_topic_root}/odom'),
            ('imu', f'{zed_topic_root}/imu/data'),
        ],
        # No --delete_db_on_start here: we reuse the mapping database
    )
    actions.append(rtabmap_localization)

    # ── 3. Nav2 bringup ──
    nav2_params = LaunchConfiguration('nav2_params').perform(context)
    map_file = LaunchConfiguration('map').perform(context)

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py',
            )
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'autostart': 'true',
        }.items(),
    )
    actions.append(nav2_bringup)

    # ── 4. RViz ──
    rviz_config = os.path.join(pkg_dir, 'rviz', 'rtabmap_nav.rviz')
    rviz = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen',
    )
    actions.append(rviz)

    return actions


def generate_launch_description():
    pkg_dir = get_local_share_dir()

    # Default Nav2 params from mina_navigation
    default_nav2_params = ''
    try:
        default_nav2_params = os.path.join(
            get_package_share_directory('mina_navigation'), 'param', 'nav2_params.yaml'
        )
    except PackageNotFoundError:
        default_nav2_params = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))))),
            'mina_navigation', 'param', 'nav2_params.yaml',
        )

    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='zed2i'),
        DeclareLaunchArgument('serial_number', default_value='38462249'),
        DeclareLaunchArgument('zed_node_name', default_value='zed_node'),
        DeclareLaunchArgument('launch_zed_wrapper', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('map', description='Path to the 2D map YAML file'),
        DeclareLaunchArgument('nav2_params', default_value=default_nav2_params,
                              description='Path to Nav2 parameters file'),
        OpaqueFunction(function=launch_setup),
    ])
