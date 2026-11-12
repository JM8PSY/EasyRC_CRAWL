"""センサー: RealSense D455（IMU・点群）と YDLIDAR X4 PRO（/scan）。

- D455 の TF は realsense2_camera が camera_link 以下（光学系・IMU フレーム）を出す。
  camera_link の親（base_link）は URDF（robot_state_publisher）が出すので、ここでは出さない。
- トピック: /camera/camera/imu, /camera/camera/depth/color/points, /scan
- X4 PRO のドライバは /scan_raw に出し、cugo_perception の scan_inf が「反射なし」の 0 を +inf に
  直して /scan に出す（ドライバの invalid_range_is_inf は使われていないため）
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_camera = LaunchConfiguration('use_camera')
    use_lidar = LaunchConfiguration('use_lidar')
    lidar_port = LaunchConfiguration('lidar_port')
    depth_profile = LaunchConfiguration('depth_profile')
    color_profile = LaunchConfiguration('color_profile')

    lidar_params = PathJoinSubstitution(
        [FindPackageShare('cugo_bringup'), 'config', 'ydlidar_x4pro.yaml'])

    realsense = Node(
        condition=IfCondition(use_camera),
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        name='camera',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'camera_name': 'camera',
            'base_frame_id': 'link',          # → camera_link（URDF の camera_link に接続）
            'publish_tf': True,
            'tf_publish_rate': 0.0,           # 静的 TF のみ
            'enable_color': True,
            'enable_depth': True,
            'enable_infra1': False,
            'enable_infra2': False,
            # Orin Nano 向けに軽く: 640x360 @ 15fps
            'depth_module.depth_profile': depth_profile,
            'rgb_camera.color_profile': color_profile,
            'enable_gyro': True,
            'enable_accel': True,
            'unite_imu_method': 2,            # linear_interpolation → /camera/camera/imu
            # → /camera/camera/depth/color/points
            # ARM（Jetson）版の librealsense ではフィルタ名が "Pointcloud (NEON)" になり、
            # パラメータ名が pointcloud__neon_.* になる。x86 用に pointcloud.* も併記
            'pointcloud__neon_.enable': True,
            'pointcloud__neon_.ordered_pc': False,
            'pointcloud.enable': True,
            'pointcloud.ordered_pc': False,
            # 点群を間引いて CPU 負荷とコストマップ処理を軽くする（320x180 相当）
            'decimation_filter.enable': True,
            'decimation_filter.filter_magnitude': 2,
            'clip_distance': 4.0,             # 4m 以遠の深度は使わない
            'align_depth.enable': False,
            'initial_reset': False,
        }],
    )

    lidar = Node(
        condition=IfCondition(use_lidar),
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_params, {'port': lidar_port}],
        remappings=[('scan', 'scan_raw')],
    )

    scan_inf = Node(
        condition=IfCondition(use_lidar),
        package='cugo_perception',
        executable='scan_inf',
        name='scan_inf',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/cugo_lidar',
            description='X4 PRO のシリアル。/dev/ydlidar はこの Jetson では ESP32 を指すので使わない'),
        DeclareLaunchArgument('depth_profile', default_value='640,360,15'),
        DeclareLaunchArgument('color_profile', default_value='640,360,15'),
        realsense,
        lidar,
        scan_inf,
    ])
