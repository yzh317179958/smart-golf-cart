"""全传感器集成 launch 文件

一键启动: WheelTec底盘 + N10P LiDAR + S11摄像头 + S11 TF + GPS + H30 IMU + GPS EKF + foxglove_bridge
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def generate_launch_description():
    wheeltec_dir = get_package_share_directory('turn_on_wheeltec_robot')
    golf_bringup_dir = get_package_share_directory('golf_bringup')

    # S11 摄像头：包缺失或launch文件断链时自动跳过，不崩溃
    s11_ok = False
    try:
        lx_camera_dir = get_package_share_directory('lx_camera_ros')
        _cam = os.path.join(lx_camera_dir, 'launch', 'lx_camera_ros.launch.py')
        s11_ok = os.path.isfile(_cam) and os.path.getsize(_cam) > 0
    except PackageNotFoundError:
        pass

    actions = [
        # S11 SDK 库路径
        SetEnvironmentVariable('LD_LIBRARY_PATH',
            '/opt/MRDVS/lib/linux_aarch64:' + os.environ.get('LD_LIBRARY_PATH', '')),

        # 1. WheelTec 底盘 (含 EKF + IMU + odom + URDF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(wheeltec_dir, 'launch', 'turn_on_wheeltec_robot.launch.py')
            )
        ),

        # 2. N10P 2D LiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(wheeltec_dir, 'launch', 'wheeltec_lidar.launch.py')
            )
        ),
    ]

    # 3-5. S11 摄像头 + TF + 深度扫描（可选，缺失自动跳过）
    if s11_ok:
        actions += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(lx_camera_dir, 'launch', 'lx_camera_ros.launch.py'))),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(golf_bringup_dir, 'launch', 's11_tf.launch.py'))),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(golf_bringup_dir, 'launch', 'depth_scan.launch.py'))),
        ]
    else:
        import sys
        print('[WARN] S11 摄像头不可用，跳过（LiDAR仍可避障）', file=sys.stderr)

    actions += [

        # 6. G90 双天线 GPS (NMEA → /gps/fix, /heading, /heading_deg)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('wheeltec_gps_driver'),
                             'launch', 'wheeltec_dual_rtk_driver_nmea.launch.py')
            )
        ),

        # 7. H30 IMU (YeSense, 9轴含磁力计, 装车顶远离电机)
        # ⚠️ yesense 驱动直接透传 NED 四元数，需经 imu_ned_to_enu 转换为 ENU
        # 数据流：yesense → /h30_imu_raw (NED) → imu_ned_to_enu → /h30_imu (ENU)
        Node(
            package='yesense_std_ros2',
            executable='yesense_node_publisher',
            name='yesense_pub',
            parameters=[
                os.path.join(get_package_share_directory('yesense_std_ros2'),
                             'config', 'yesense_config.yaml'),
                {'imu_topic_ros': '/h30_imu_raw'},  # NED 原始数据
            ],
            output='screen',
        ),

        # 7b. H30 NED→ENU 转换 + G90 双天线航向融合
        # G90 双天线航向（/heading）可用时直接使用（±0.3°）
        # G90 不可用时回退 H30 磁力计（yaw_offset=-1.988）
        Node(
            package='golf_mapping',
            executable='imu_ned_to_enu',
            name='imu_ned_to_enu',
            output='screen',
        ),

        # 8. GPS 天线静态 TF (base_footprint → navsat_link，GPS 天线在车顶中心)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.5', '0', '0', '0',
                       'base_footprint', 'navsat_link'],
            name='gps_tf',
        ),

        # 9. GPS融合EKF（ODOM模式：GPS绝对修正 + G90航向）
        # 输出 /odom_gps_fused（GPS修正+G90航向+里程计平滑），publish_tf=false避免与EKF#1冲突
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_gps_node',
            parameters=[os.path.join(golf_bringup_dir, 'config', 'ekf_gps.yaml')],
            remappings=[
                ('/odometry/filtered', '/odom_gps_fused'),  # GPS融合输出（非/odom_combined，避免与EKF#1冲突）
            ],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

        # 10. navsat_transform_node: GPS lat/lon → UTM → /odometry/gps + 反算 /gps/filtered
        # 读 /odom_gps_fused（GPS融合EKF输出，含G90航向）→ 反算出平滑的 /gps/filtered
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[os.path.join(golf_bringup_dir, 'config', 'ekf_gps.yaml')],
            remappings=[
                ('/imu', '/h30_imu'),
                ('/odometry/filtered', '/odom_gps_fused'),  # ★ 读GPS融合EKF输出（含G90航向）
            ],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

        # 11. foxglove_bridge (Foxglove 可视化，轻量无内存泄漏)
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(get_package_share_directory('foxglove_bridge'),
                             'launch', 'foxglove_bridge_launch.xml')
            ),
            launch_arguments={'port': '8765'}.items(),
        ),
    ]

    return LaunchDescription(actions)
