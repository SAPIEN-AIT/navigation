import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # 2. Path resolutions
    pkg_share = get_package_share_directory('mina_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'berkeley_humanoid_lite.urdf')

    # Read the URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    
    # 3. Nodes
    
    # Publishes the TFs of the robot based on the URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    # Publishes default (zero) joint states for revolute/continuous joints
    # This prevents RViz TF errors when a real robot/simulator is not running yet
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
        joint_state_publisher_node
    ])