import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def get_local_share_dir():
    try:
        return get_package_share_directory('zed2i_isaac_vslam')
    except Exception:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    pkg = get_local_share_dir()
    slam_params = os.path.join(pkg, 'config', 'slam_toolbox_params.yaml')

    camera_name    = LaunchConfiguration('camera_name',    default='zed2i')
    zed_node_name  = LaunchConfiguration('zed_node_name',  default='zed_node')
    scan_height    = LaunchConfiguration('scan_height',    default='5')
    range_min      = LaunchConfiguration('range_min',      default='0.3')
    range_max      = LaunchConfiguration('range_max',      default='8.0')

    depth_topic     = ['/'],   camera_name, ['/', zed_node_name, '/depth/depth_registered']
    cam_info_topic  = ['/'],   camera_name, ['/', zed_node_name, '/depth/camera_info']

    # depthimage_to_laserscan: converts ZED depth image → virtual LaserScan
    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_to_laserscan',
        parameters=[{
            'scan_height':    scan_height,
            'range_min':      range_min,
            'range_max':      range_max,
            'output_frame':   [camera_name, '_left_camera_optical_frame'],
        }],
        remappings=[
            ('depth',       ['/', camera_name, '/', zed_node_name, '/depth/depth_registered']),
            ('depth_camera_info', ['/', camera_name, '/', zed_node_name, '/depth/camera_info']),
            ('scan',        '/scan_virtual'),
        ],
    )

    # slam_toolbox: builds 2D occupancy grid map
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params],
        output='screen',
    )

    # RViz
    rviz_config = os.path.join(pkg, 'rviz', 'zed2i_mapping.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapping',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_name',   default_value='zed2i'),
        DeclareLaunchArgument('zed_node_name', default_value='zed_node'),
        DeclareLaunchArgument('scan_height',   default_value='5'),
        DeclareLaunchArgument('range_min',     default_value='0.3'),
        DeclareLaunchArgument('range_max',     default_value='8.0'),
        depth_to_scan,
        slam,
        rviz,
    ])
