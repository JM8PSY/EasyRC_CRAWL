"""twist_mux: /cmd_vel_teleop（優先）と Nav2 の /cmd_vel_nav_safe を /cmd_vel にまとめる。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config = PathJoinSubstitution(
        [FindPackageShare('cugo_bringup'), 'config', 'twist_mux.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel_out', '/cmd_vel')],
        ),
    ])
