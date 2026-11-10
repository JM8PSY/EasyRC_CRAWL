"""robot_state_publisher で CuGo V3 の URDF を配信する。

センサー取付位置などは xacro 引数で上書きできる（既定値は仮・要実測）。
例: ros2 launch cugo_description description.launch.py \
      xacro_args:="lidar_x:=0.05 lidar_z:=0.25"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    xacro_args = LaunchConfiguration('xacro_args')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('cugo_description'), 'urdf', 'cugo.urdf.xacro'])
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' ', xacro_args]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'xacro_args', default_value='',
            description='xacro に渡す引数（例 "lidar_z:=0.25 camera_x:=0.24"）'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
