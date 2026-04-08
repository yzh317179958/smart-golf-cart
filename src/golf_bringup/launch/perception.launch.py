"""yolo_ros 感知层启动文件

启动 YOLO 检测（手势模型 person/ok/stop）+ ByteTrack 跟踪，
适配 S11 相机话题。
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 模型路径（TensorRT FP16 引擎）
    model_path = os.path.expanduser('~/golf_ws/models/yolov8s-pose.engine')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('yolo_bringup'),
                    'launch', 'yolo.launch.py'
                ])
            ]),
            launch_arguments={
                'model': model_path,
                'input_image_topic': '/lx_camera_node/LxCamera_Rgb',
                'input_depth_topic': '/lx_camera_node/LxCamera_Depth',
                'input_depth_info_topic': '/lx_camera_node/LxCamera_TofInfo',
                'depth_image_units_divisor': '1000',
                'target_frame': 'base_link',
                'imgsz_height': '640',
                'imgsz_width': '640',
                'threshold': '0.5',
                'device': 'cuda:0',
                'enable': 'True',
                'use_tracking': 'True',
                'use_3d': 'False',
                'use_debug': 'False',  # 节省 ~27% CPU，需要可视化时改 True
                'image_reliability': '1',
                'depth_image_reliability': '1',
                'depth_info_reliability': '1',
                'namespace': 'yolo',
            }.items()
        ),
    ])
