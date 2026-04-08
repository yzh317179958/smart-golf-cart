"""S11 摄像头静态 TF 发布 launch 文件

发布 base_link → s11_camera_link 的静态变换。
安装位置参考 WheelTec URDF camera_link：前方 0.312m，高 0.135m。
后续根据实测调整。
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # S11 摄像头静态 TF: base_link → s11_camera_link
        # 参数顺序: x y z yaw pitch roll parent child
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='s11_static_tf',
            arguments=[
                '--x', '0.312',
                '--y', '0.0',
                '--z', '0.135',
                '--yaw', '0.0',
                '--pitch', '0.0',
                '--roll', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 's11_camera_link',
            ],
        ),
        # S11 光学坐标系 (ROS 标准: Z 前, X 右, Y 下)
        # camera_link → camera_optical_link 旋转 -90° yaw, -90° pitch
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='s11_optical_tf',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--yaw', '-1.5708',
                '--pitch', '0.0',
                '--roll', '-1.5708',
                '--frame-id', 's11_camera_link',
                '--child-frame-id', 's11_camera_optical_link',
            ],
        ),
    ])
