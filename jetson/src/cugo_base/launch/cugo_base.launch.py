"""cugo_base_node を起動する。

例: ros2 launch cugo_base cugo_base.launch.py port:=/dev/cugo_esp32
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory('cugo_base'), 'config', 'cugo_base.yaml')
    params_file = LaunchConfiguration('params_file')
    port = LaunchConfiguration('port')
    use_stamped = LaunchConfiguration('use_stamped')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='cugo_base_node のパラメータ YAML'),
        DeclareLaunchArgument('port', default_value='/dev/cugo_esp32',
                              description='ESP32 のシリアルポート'),
        DeclareLaunchArgument('use_stamped', default_value='false',
                              description='/cmd_vel を TwistStamped で購読する'),
        Node(
            package='cugo_base',
            executable='cugo_base_node',
            name='cugo_base_node',
            output='screen',
            parameters=[params_file, {'port': port, 'use_stamped': use_stamped}],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
