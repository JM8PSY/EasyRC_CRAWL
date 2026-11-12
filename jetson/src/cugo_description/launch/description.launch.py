"""robot_state_publisher で CuGo V3 の URDF を配信する。

センサー取付位置などは xacro 引数で上書きできる（既定値は仮・要実測）。
例: ros2 launch cugo_description description.launch.py \
      xacro_args:="lidar_x:=0.05 lidar_z:=0.25"

実機で調整した値は ~/.config/cugo/calibration.yaml（例: `lidar_yaw: -1.5708`）に保存され、
起動時に xacro 引数として渡される（Web UI の「LiDAR補正」が書き込む）。xacro_args の指定が優先。
"""
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

CALIBRATION_FILE = os.path.expanduser('~/.config/cugo/calibration.yaml')
# calibration.yaml で上書きできる xacro 引数
CALIBRATION_KEYS = ('lidar_yaw',)


def calibration_args(path=CALIBRATION_FILE):
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        return ''
    args = []
    for key in CALIBRATION_KEYS:
        value = data.get(key) if isinstance(data, dict) else None
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            args.append('%s:=%r' % (key, float(value)))
    return ' '.join(args)


def _robot_state_publisher(context):
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('cugo_description'), 'urdf', 'cugo.urdf.xacro'])
    calib = calibration_args()
    if calib:
        print('[cugo_description] calibration: %s (%s)' % (calib, CALIBRATION_FILE))
    # 後ろの引数が優先されるので、xacro_args（手動指定）を最後に置く
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' ', calib, ' ', LaunchConfiguration('xacro_args')]),
        value_type=str)
    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'xacro_args', default_value='',
            description='xacro に渡す引数（例 "lidar_z:=0.25 camera_x:=0.24"）'),
        OpaqueFunction(function=_robot_state_publisher),
    ])
