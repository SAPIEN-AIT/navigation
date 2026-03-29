import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Force simulation (Isaac Sim) clock'
    )

    # 2. Path resolutions
    bringup_pkg_share = get_package_share_directory('mina_bringup')
    control_pkg_share = get_package_share_directory('mina_control')
    simulation_pkg_share = get_package_share_directory('mina_simulation')
    description_pkg_share = get_package_share_directory('mina_description')
    
    onnx_policy_path = os.path.join(control_pkg_share, 'policies', 'model.onnx')
    isaac_launch_path = os.path.join(simulation_pkg_share, 'launch', 'isaac_sim.launch.py')
    rsp_launch_path = os.path.join(description_pkg_share, 'launch', 'mina_rsp.launch.py')
    
    # Path to the RViz configuration file
    rviz_config_path = os.path.join(bringup_pkg_share, 'rviz', 'mina_visual.rviz')

    # 3. Environment and Robot Structure
    isaac_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(isaac_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Load the robot state publisher
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. Control Nodes
    fsm_node = Node(
        package='mina_control',
        executable='mina_fsm',
        name='mina_fsm',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    dynamic_switch_node = Node(
        package='mina_control',
        executable='dynamic_switch',
        name='dynamic_switch',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'control_sources': ['policy_1'],
            'transition_duration': 0.5,
            'max_safe_delta': 0.2
            }]
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

    # 5. Teleoperation Nodes
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    teleop_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[{
            'use_sim_time': use_sim_time,
            'require_enable_button': False
        }]
    )

    # 6. Visualization
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_path, '--ros-args', '--log-level', 'FATAL'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        # isaac_sim_launch,
        rsp_launch,
        fsm_node,
        dynamic_switch_node,
        rl_controller_node,
        joy_node,
        teleop_joy_node,
        rviz2_node,
    ])