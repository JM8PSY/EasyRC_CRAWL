"""CuGo V3 一括起動。

  地図作り + Nav2:   ros2 launch cugo_bringup bringup.launch.py mode:=slam
  保存地図で Nav2:   ros2 launch cugo_bringup bringup.launch.py mode:=nav map:=$HOME/maps/room.yaml

起動するもの: robot_state_publisher（URDF）、cugo_base（ESP32 ブリッジ）、センサー（D455 / X4 PRO）、
rf2o + EKF、slam_toolbox（slam）または map_server + AMCL（nav）、Nav2、twist_mux、（任意で）RViz。
use_base:=false で cugo_base を起動しない（モーターに一切つながない。センサーと計算だけの確認用）。
use_nav:=false で Nav2 を起動しない（手動で走らせて地図だけ作るとき）。
Web UI: ブラウザで http://<JetsonのIP>:8080/（use_web:=false で起動しない）。
下り段差検出（cugo_perception の cliff_detector）: use_cliff:=false で起動しない。
RICOH THETA の床検出（theta_floor、自動探索の補助）: use_theta:=false で起動しない。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(pkg, launch_file, condition=None, args=None):
    # include した launch の DeclareLaunchArgument は外に漏れる（例: cugo_base の params_file が
    # Nav2 の params_file を上書きして Nav2 が cugo_base.yaml を読んでしまう）。
    # GroupAction（scoped）で include ごとに閉じ込める。
    return GroupAction(
        [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare(pkg), 'launch', launch_file])),
            launch_arguments=(args or {}).items(),
        )],
        condition=condition,
        scoped=True,
    )


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    use_base = LaunchConfiguration('use_base')
    use_nav = LaunchConfiguration('use_nav')
    base_port = LaunchConfiguration('base_port')
    lidar_port = LaunchConfiguration('lidar_port')
    use_web = LaunchConfiguration('use_web')
    use_cliff = LaunchConfiguration('use_cliff')
    use_theta = LaunchConfiguration('use_theta')
    # Web UI（自動探索）へそのまま渡す。include は scoped なので、ここで受けて渡す必要がある
    web_args = {k: LaunchConfiguration(k) for k in
                ('use_camera_floor', 'explore_log', 'explore_log_dir', 'explore_variant')}

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
        DeclareLaunchArgument('use_web', default_value='true',
                              description='Web UI（http://<JetsonのIP>:8080/）を起動する'),
        DeclareLaunchArgument('use_cliff', default_value='true',
                              description='RealSense の点群で下り段差（階段）を検出し Nav2 の障害物にする'),
        DeclareLaunchArgument('use_theta', default_value='true',
                              description='RICOH THETA で前方の床の続きを見て自動探索の判断に使う'),
        DeclareLaunchArgument('use_camera_floor', default_value='true',
                              description='カメラ（D455 / THETA）が床と見たセルを自動探索の判断で空きにする'),
        DeclareLaunchArgument('explore_log', default_value='true',
                              description='自動探索を 1 回ごとに explore_log_dir/explore_*.jsonl に記録'),
        DeclareLaunchArgument('explore_log_dir', default_value='~/cugo_logs'),
        DeclareLaunchArgument('explore_variant', default_value='',
                              description='記録に残す方法の名前（explog_report.py で比べるとき用）'),
        DeclareLaunchArgument('base_port', default_value='/dev/cugo_esp32'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/cugo_lidar'),

        _include('cugo_description', 'description.launch.py'),
        _include('cugo_base', 'cugo_base.launch.py', IfCondition(use_base),
                 {'port': base_port, 'use_stamped': 'false'}),
        _include('cugo_bringup', 'sensors.launch.py', args={'lidar_port': lidar_port}),
        _include('cugo_bringup', 'odometry.launch.py'),
        _include('cugo_perception', 'cliff_detector.launch.py', IfCondition(use_cliff)),
        _include('cugo_perception', 'theta_floor.launch.py', IfCondition(use_theta),
                 {'params_file': PathJoinSubstitution(
                     [FindPackageShare('cugo_bringup'), 'config', 'theta.yaml'])}),
        _include('cugo_bringup', 'slam.launch.py', is_slam),
        _include('cugo_bringup', 'localization.launch.py', is_nav, {'map': map_yaml}),
        _include('cugo_bringup', 'navigation.launch.py', IfCondition(use_nav)),
        _include('cugo_bringup', 'twist_mux.launch.py'),
        _include('cugo_web', 'cugo_web.launch.py', IfCondition(use_web),
                 dict({'map_yaml': map_yaml}, **web_args)),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='log',
        ),
    ])
