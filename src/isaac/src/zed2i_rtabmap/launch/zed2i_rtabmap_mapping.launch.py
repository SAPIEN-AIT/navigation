"""
RTABMap mapping with ZED2i camera.

This launch file starts:
  1. ZED2i camera wrapper (with odometry enabled)
  2. RTABMap in MAPPING mode (SLAM + 2D occupancy grid)
  3. RViz for visualization

Usage:
  ros2 launch zed2i_rtabmap zed2i_rtabmap_mapping.launch.py

After mapping, save the 2D map for Nav2:
  ros2 run nav2_map_server map_saver_cli -f ~/my_map
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

    # ── 2. RTABMap SLAM node ──
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_params],
        remappings=[
            ('rgb/image', f'{zed_topic_root}/rgb/image_rect_color'),
            ('depth/image', f'{zed_topic_root}/depth/depth_registered'),
            ('rgb/camera_info', f'{zed_topic_root}/rgb/camera_info'),
            ('odom', f'{zed_topic_root}/odom'),
            ('imu', f'{zed_topic_root}/imu/data'),
        ],
        arguments=['--delete_db_on_start'],
    )
    actions.append(rtabmap_node)

    # ── 3. RTABMap visualization node (publishes 3D map) ──
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rtabmap_viz')),
        parameters=[rtabmap_params],
        remappings=[
            ('rgb/image', f'{zed_topic_root}/rgb/image_rect_color'),
            ('depth/image', f'{zed_topic_root}/depth/depth_registered'),
            ('rgb/camera_info', f'{zed_topic_root}/rgb/camera_info'),
            ('odom', f'{zed_topic_root}/odom'),
        ],
    )
    actions.append(rtabmap_viz)

    # ── 4. RViz ──
    rviz_config = os.path.join(pkg_dir, 'rviz', 'rtabmap_mapping.rviz')
    rviz = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2_rtabmap',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen',
    )
    actions.append(rviz)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='zed2i'),
        DeclareLaunchArgument('serial_number', default_value='38462249'),
        DeclareLaunchArgument('zed_node_name', default_value='zed_node'),
        DeclareLaunchArgument('launch_zed_wrapper', default_value='true',
                              description='Launch ZED wrapper (set false if already running)'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_rtabmap_viz', default_value='false',
                              description='Launch RTABMap built-in visualizer'),
        OpaqueFunction(function=launch_setup),
    ])
