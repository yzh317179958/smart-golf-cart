"""P2-15: 精简版 Nav2 launch（仅 MPPI + local_costmap）

只启动 controller_server（MPPI 局部避障）+ lifecycle_manager。
不启动 bt_navigator / planner_server / behavior_server / map_server / amcl。
用 Nav2 的 20%，自研替代其余 80%。

controller_server 输出 remap: /cmd_vel → /cmd_vel_nav
LiDAR 急停层（P2-11）门控: /cmd_vel_nav → /cmd_vel → 底盘

前提: sensors.launch.py 已启动（底盘+LiDAR+TF+EKF）
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    golf_bringup_dir = get_package_share_directory('golf_bringup')
    nav2_params_file = os.path.join(golf_bringup_dir, 'config', 'nav2_params.yaml')

    params_file = LaunchConfiguration('params_file')

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={'use_sim_time': 'false'},
        convert_types=True,
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_params_file,
            description='Nav2 参数文件',
        ),

        # controller_server: MPPI + local_costmap
        # 输出 remap 到 /cmd_vel_nav（LiDAR 急停门控后输出到 /cmd_vel）
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=[('/cmd_vel', '/cmd_vel_nav')],
        ),

        # lifecycle_manager: 只管 controller_server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mppi',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['controller_server'],
            }],
        ),
    ])
