"""CuGo Web UI。ブラウザで http://<JetsonのIP>:8080/ を開く。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    map_yaml = LaunchConfiguration('map_yaml')
    # 自動探索の比較用（explog.py。~/cugo_logs/explore_*.jsonl）
    explore = {k: LaunchConfiguration(k) for k in
               ('use_camera_floor', 'explore_log', 'explore_log_dir', 'explore_variant')}
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('map_yaml', default_value='',
                              description='保存地図で起動したときの地図 YAML（<地図>.keepout.yaml を使う）'),
        DeclareLaunchArgument('use_camera_floor', default_value='true',
                              description='カメラが床と見たセルを自動探索の判断で空きにする'),
        DeclareLaunchArgument('explore_log', default_value='true',
                              description='自動探索を 1 回ごとに JSONL に記録する'),
        DeclareLaunchArgument('explore_log_dir', default_value='~/cugo_logs'),
        DeclareLaunchArgument('explore_variant', default_value='',
                              description='記録に残す方法の名前（比べるとき用。例: cam_off）'),
        Node(
            package='cugo_web',
            executable='cugo_web_node',
            name='cugo_web',
            output='screen',
            parameters=[dict({'port': port, 'map_yaml': map_yaml}, **explore)],
        ),
    ])
