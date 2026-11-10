"""CuGo V3 一括起動。

  地図作り + Nav2:   ros2 launch cugo_bringup bringup.launch.py mode:=slam
  保存地図で Nav2:   ros2 launch cugo_bringup bringup.launch.py mode:=nav map:=$HOME/maps/room.yaml

起動するもの: robot_state_publisher（URDF）、cugo_base（ESP32 ブリッジ）、センサー（D455 / X4 PRO）、
rf2o + EKF、slam_toolbox（slam）または map_server + AMCL（nav）、Nav2、twist_mux、（任意で）RViz。
use_base:=false で cugo_base を起動しない（モーターに一切つながない。センサーと計算だけの確認用）。
use_nav:=false で Nav2 を起動しない（手動で走らせて地図だけ作るとき）。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(pkg, launch_file, condition=None, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), 'launch', launch_file])),
        condition=condition,
        launch_arguments=(args or {}).items(),
    )


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    use_base = LaunchConfiguration('use_base')
    use_nav = LaunchConfiguration('use_nav')
    base_port = LaunchConfiguration('base_port')
    lidar_port = LaunchConfiguration('lidar_port')

    is_slam = IfCondition(EqualsSubstitution(mode, 'slam'))
    is_nav = IfCondition(EqualsSubstitution(mode, 'nav'))

    rviz_config = PathJoinSubstitution(
        [FindPackageShare('cugo_bringup'), 'rviz', 'cugo.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='slam', choices=['slam', 'nav'],
                              description='slam: 地図を作りながら移動 / nav: 保存した地図で AMCL'),
        DeclareLaunchArgument('map', default_value='',
                              description='mode:=nav で使う地図 YAML のフルパス'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_base', default_value='true',
                              description='false で cugo_base（ESP32 ブリッジ）を起動しない'),
        DeclareLaunchArgument('use_nav', default_value='true',
                              description='false で Nav2 を起動しない'),
        DeclareLaunchArgument('base_port', default_value='/dev/cugo_esp32'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/cugo_lidar'),

        _include('cugo_description', 'description.launch.py'),
        _include('cugo_base', 'cugo_base.launch.py', IfCondition(use_base),
                 {'port': base_port, 'use_stamped': 'false'}),
        _include('cugo_bringup', 'sensors.launch.py', args={'lidar_port': lidar_port}),
        _include('cugo_bringup', 'odometry.launch.py'),
        _include('cugo_bringup', 'slam.launch.py', is_slam),
        _include('cugo_bringup', 'localization.launch.py', is_nav, {'map': map_yaml}),
        _include('cugo_bringup', 'navigation.launch.py', IfCondition(use_nav)),
        _include('cugo_bringup', 'twist_mux.launch.py'),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='log',
        ),
    ])
