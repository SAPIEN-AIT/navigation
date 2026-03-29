import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch argument to be passed from the bringup package
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Isaac Sim) clock if true'
    )

    # Create a LaunchConfiguration variable to pass to the nodes
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Resolve the path to the ONNX policy inside the 'policies' directory
    pkg_share = get_package_share_directory('mina_control')
    onnx_policy_path = os.path.join(pkg_share, 'policies', 'model.onnx')

    # Define the nodes
    fsm_node = Node(
        package='mina_control',
        executable='mina_fsm',
        name='mina_fsm',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rl_controller_node = Node(
        package='mina_control',
        executable='rl_controller',
        name='rl_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'policy_path': onnx_policy_path
        }]
    )

    vmc_node = Node(
        package='mina_control',
        executable='mina_vmc',
        name='mina_vmc',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    dynamic_switch_node = Node(
        package='mina_control',
        executable='dynamic_switch',
        name='dynamic_switch',
        output='screen',
        parameters=[{
                'control_sources': ['policy_1'],
                'transition_duration': 0.5,
                'max_safe_delta': 0.2
            }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        fsm_node,
        rl_controller_node,
        vmc_node,
        dynamic_switch_node,
    ])