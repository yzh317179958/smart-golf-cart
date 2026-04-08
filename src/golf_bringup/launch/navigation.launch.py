"""Nav2 导航 launch 文件

直接复用 WheelTec 官方 bringup_launch.py（包含 map_server + AMCL + Nav2 全栈）
前提：sensors.launch.py 已启动（底盘、LiDAR、TF、EKF）
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    golf_bringup_dir = get_package_share_directory('golf_bringup')
    nav2_params_file = os.path.join(golf_bringup_dir, 'config', 'nav2_params.yaml')

    # 默认地图文件路径
    default_map = '/home/wheeltec/golf_ws/map/test_map.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='地图文件路径 (yaml)',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_params_file,
            description='Nav2 参数文件路径',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用仿真时间',
        ),

        # 直接复用 WheelTec 官方 bringup_launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(golf_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'slam': 'False',
                'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file'),
                'autostart': 'true',
            }.items(),
        ),
    ])
