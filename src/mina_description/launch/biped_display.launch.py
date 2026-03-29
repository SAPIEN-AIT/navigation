import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('mina_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'berkeley_humanoid_lite_biped.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'mina.rviz')

    # Read the URDF file
    with open(default_model_path, 'r') as infp:
        robot_desc = infp.read()
    
    # Publishes the TFs of the robot based on the URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    # Opens a GUI with sliders to manually move the robot's joints
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # Starts RViz2 with the specific configuration for the URDF
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])