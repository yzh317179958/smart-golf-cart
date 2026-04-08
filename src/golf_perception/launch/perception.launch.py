"""感知节点 launch 文件

启动 YOLOv8-Pose 检测节点
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='/root/golf_ws/src/golf_perception/models/yolov8s-pose.engine',
            description='YOLOv8-Pose TensorRT 引擎路径',
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.5',
            description='检测置信度阈值',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/lx_camera_node/LxCamera_Rgb',
            description='输入 RGB 图像话题',
        ),

        Node(
            package='golf_perception',
            executable='detection_node',
            name='detection_node',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'image_topic': LaunchConfiguration('image_topic'),
            }],
            output='screen',
        ),
    ])
