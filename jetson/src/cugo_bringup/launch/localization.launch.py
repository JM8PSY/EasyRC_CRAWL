"""保存した地図で自己位置推定（map_server + AMCL）。/map と map → odom を出す。

例: ros2 launch cugo_bringup localization.launch.py map:=$HOME/maps/room.yaml
初期位置は RViz の「2D Pose Estimate」で与える。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('nav2_bringup'), 'launch', 'localization_launch.py'])),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', description='地図の YAML（map_saver で保存したもの）のフルパス'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('cugo_bringup'), 'config', 'nav2_params.yaml'])),
        localization,
    ])
