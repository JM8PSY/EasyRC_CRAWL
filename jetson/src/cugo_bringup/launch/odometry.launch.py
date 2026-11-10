"""オドメトリ: rf2o_laser_odometry（/scan → /odom_rf2o）+ robot_localization EKF。

EKF が odom → base_footprint の TF と /odometry/filtered を出す。
rf2o は起動時に base_footprint → laser_frame の TF を読むので、robot_state_publisher を先に起動しておくこと。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_params = PathJoinSubstitution(
        [FindPackageShare('cugo_bringup'), 'config', 'ekf.yaml'])

    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,              # TF は EKF が出す
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,                     # X4 PRO のスキャン周期に合わせる
            'use_sim_time': use_sim_time,
        }],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        rf2o,
        ekf,
    ])
