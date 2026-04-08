"""slam_toolbox 2D SLAM launch 文件

支持两种模式:
  - mapping: 建图模式（默认）
  - localization: 定位模式（需提供 map_file_name）

用法:
  ros2 launch golf_bringup slam.launch.py
  ros2 launch golf_bringup slam.launch.py mode:=localization map_file_name:=/home/wheeltec/golf_ws/map/test_map_serialized
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def launch_setup(context):
    pkg_dir = get_package_share_directory('golf_bringup')
    slam_config = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')

    mode = LaunchConfiguration('mode').perform(context)
    map_file_name = LaunchConfiguration('map_file_name').perform(context)

    # 根据模式选择不同的 executable（参考 WheelTec 官方示例）
    if mode == 'localization':
        executable = 'localization_slam_toolbox_node'
    else:
        executable = 'async_slam_toolbox_node'

    params = {}
    if map_file_name:
        params['map_file_name'] = map_file_name
        if mode == 'localization':
            # localization 模式不支持 map_start_at_dock，用 map_start_pose
            params['map_start_pose'] = [0.0, 0.0, 0.0]
        else:
            params['map_start_at_dock'] = True

    slam_node = Node(
        package='slam_toolbox',
        executable=executable,
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, params],
    )

    return [slam_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='mapping',
            description='slam_toolbox mode: mapping or localization'
        ),
        DeclareLaunchArgument(
            'map_file_name',
            default_value='',
            description='Full path to map file (without extension) for localization mode'
        ),
        OpaqueFunction(function=launch_setup),
    ])
