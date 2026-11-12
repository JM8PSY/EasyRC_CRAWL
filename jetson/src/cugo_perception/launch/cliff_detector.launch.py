"""下り階段・段差落ち（崖）検出。/cliff/points（印）と /cliff/floor（消去）を出す。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cloud_topic = LaunchConfiguration('cloud_topic')
    return LaunchDescription([
        DeclareLaunchArgument('cloud_topic', default_value='/camera/camera/depth/color/points',
                              description='入力の点群（D455、カメラ光学座標）'),
        Node(
            package='cugo_perception',
            executable='cliff_detector',
            name='cliff_detector',
            output='screen',
            parameters=[{'cloud_topic': cloud_topic}],
        ),
    ])
