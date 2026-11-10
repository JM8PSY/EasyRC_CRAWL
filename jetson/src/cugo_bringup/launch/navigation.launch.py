"""Nav2（nav2_bringup の navigation_launch.py）を config/nav2_params.yaml で起動する。

Jazzy の navigation_launch.py は controller_server / behavior_server / velocity_smoother の
cmd_vel を /cmd_vel_nav（Nav2 内部の生の指令）にリマップしている。最終出力は
collision_monitor の cmd_vel_out_topic（/cmd_vel_nav_safe）で、twist_mux が /cmd_vel にまとめる。

注意: Jazzy の navigation_launch.py は docking_server の cmd_vel をリマップしないため、そのままだと
docking_server が /cmd_vel に直接出力でき、twist_mux（teleop 優先）を迂回してしまう。
そこで include 全体に SetRemap('cmd_vel' → 'cmd_vel_nav') を掛け、docking_server の出力も
velocity_smoother → collision_monitor → twist_mux を通るようにしている（他のノードは元々 cmd_vel_nav）。
地図（/map）と map → odom は slam.launch.py か localization.launch.py が出す。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('cugo_bringup'), 'config', 'nav2_params.yaml'])),
        GroupAction([
            SetRemap(src='cmd_vel', dst='cmd_vel_nav'),
            nav2,
        ]),
    ])
