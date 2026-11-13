"""RICOH THETA で前方の床がどこまで続くかを見る（SLAM の補助）。/theta/floor を出す。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value='',
                              description='theta_floor のパラメータ（cugo_bringup/config/theta.yaml）'),
        Node(
            package='cugo_perception',
            executable='theta_floor',
            name='theta_floor',
            output='screen',
            parameters=[params],
        ),
    ])
