"""CuGo Web UI。ブラウザで http://<JetsonのIP>:8080/ を開く。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    map_yaml = LaunchConfiguration('map_yaml')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('map_yaml', default_value='',
                              description='保存地図で起動したときの地図 YAML（<地図>.keepout.yaml を使う）'),
        Node(
            package='cugo_web',
            executable='cugo_web_node',
            name='cugo_web',
            output='screen',
            parameters=[{'port': port, 'map_yaml': map_yaml}],
        ),
    ])
