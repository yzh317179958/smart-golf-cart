"""S11 深度图 → 虚拟 LaserScan（前方深度避障）

将 S11 ToF 深度图转为 2D LaserScan，注入 Nav2 Costmap 做前方避障。
配合 N10P 2D LiDAR（360° 平面）实现更完整的障碍物检测。

S11 ToF 深度图: 240x90, mono16 (mm), frame_id: mrdvs_tof
S11 TofInfo:     frame_id: intrinsic_tof, distortion_model: LX_DISTORTION_FISHEYE (非标准)

注意: depthimage_to_laserscan 不认识 LX_DISTORTION_FISHEYE 畸变模型，
需要用 camera_info_republisher 节点将 distortion_model 改为 plumb_bob。
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # S11 深度 frame TF 补充:
        # mrdvs_tof → s11_camera_link (深度图 frame 挂到我们的 TF 树)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mrdvs_tof_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                '--frame-id', 's11_camera_link',
                '--child-frame-id', 'mrdvs_tof',
            ],
        ),
        # intrinsic_tof 也挂上（camera_info 用的 frame）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='intrinsic_tof_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                '--frame-id', 's11_camera_link',
                '--child-frame-id', 'intrinsic_tof',
            ],
        ),

        # camera_info 修复: 将非标准 LX_DISTORTION_FISHEYE → plumb_bob
        # depthimage_to_laserscan 不支持自定义畸变模型
        Node(
            package='golf_perception',
            executable='camera_info_fix',
            name='camera_info_fix',
            parameters=[{
                'input_topic': '/lx_camera_node/LxCamera_TofInfo',
                'output_topic': '/depth_camera_info_fixed',
            }],
        ),

        # depthimage_to_laserscan: 深度图 → 虚拟 LaserScan
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depth_to_scan',
            parameters=[{
                'scan_time': 0.067,        # ~15fps
                'range_min': 0.3,          # S11 ToF 最小有效距离
                'range_max': 8.0,          # S11 ToF 最大有效距离
                'scan_height': 1,          # 深度图中心 1 行（90 行总高）
                'output_frame': 'mrdvs_tof',
            }],
            remappings=[
                ('depth', '/depth_image_fixed'),
                ('depth_camera_info', '/depth_camera_info_fixed'),
                ('scan', '/depth_scan'),
            ],
        ),
    ])
