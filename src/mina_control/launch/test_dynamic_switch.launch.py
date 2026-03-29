import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Load URDF for publishers
    pkg_share = get_package_share_directory('mina_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'berkeley_humanoid_lite.urdf')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
        
    rviz_config = os.path.join(pkg_share, 'rviz', 'mina.rviz')

    return LaunchDescription([
        # 1. Dummy Controller 1 (Policy 1)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='gui_policy_1',
            remappings=[('/joint_states', '/policy_1/gui_joints')]
        ),
        
        # 2. Dummy Controller 2 (Policy 2)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='gui_policy_2',
            remappings=[('/joint_states', '/policy_2/gui_joints')]
        ),
        
        # 3. Dynamic Switch
        Node(
            package='mina_control',
            executable='dynamic_switch',
            parameters=[{
                'control_sources': ['policy_1', 'policy_2'],
                'transition_duration': 2.0, # Slowed down to 2 seconds to see the interpolation clearly!
                # 'max_safe_delta': 3.14 # High tolerance for testing
                'max_safe_delta': 0.2 # High tolerance for testing
            }]
        ),
        
        # 4. The Adapter Node (Translates GUI <-> Switch <-> RViz)
        Node(
            package='mina_control',
            executable='switch_test_adapter'
        ),

        # 5. Robot State Publisher (Calculates TF for RViz)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),

        # 6. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config]
        )
    ])