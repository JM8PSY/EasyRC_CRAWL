"""CuGo Web UI。ブラウザで http://<JetsonのIP>:8080/ を開く。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080'),
        Node(
            package='cugo_web',
            executable='cugo_web_node',
            name='cugo_web',
            output='screen',
            parameters=[{'port': port}],
        ),
    ])
