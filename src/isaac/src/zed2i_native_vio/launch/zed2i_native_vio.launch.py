import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_name = LaunchConfiguration('camera_name')
    serial_number = LaunchConfiguration('serial_number')

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
            'serial_number': serial_number,
            'publish_urdf': 'true',
            'publish_tf': 'true',
            'publish_map_tf': 'false',
            'publish_imu_tf': 'true',
            'pos_tracking_enabled': 'true',
            'pos_tracking_mode': 'GEN_1',
        }.items(),
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='zed2i'),
        DeclareLaunchArgument('serial_number', default_value='38462249'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        zed_wrapper_launch,
        rviz_node,
    ])
