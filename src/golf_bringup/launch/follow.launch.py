"""跟随功能 launch 文件

启动九个节点：模式管理 + 手势识别 + 目标锁定 + PID跟随控制
  + collision_monitor避障 + GPS路点记录 + GPS路点导航
  + 召唤服务 + MQTT桥接
依赖：sensors.launch.py + perception.launch.py 已启动
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    golf_bringup_dir = get_package_share_directory('golf_bringup')

    return LaunchDescription([
        # 0. 模式管理器：FOLLOWING / NAVIGATION / E_STOP 三状态机
        Node(
            package='golf_navigation',
            executable='mode_manager',
            name='mode_manager',
            parameters=[{
                'lost_estop_timeout': 15.0,
                'estop_joy_button': 0,
                'resume_joy_button': 1,
            }],
            output='screen',
        ),

        # 1. 手势识别：YOLO 关键点 → lock/stop 命令
        Node(
            package='golf_perception',
            executable='gesture_node',
            name='gesture_node',
            parameters=[{
                'threshold': 15.0,
                'lock_duration': 0.5,
                'stop_duration': 1.0,
                'lock_min_frames': 3,
                'stop_min_frames': 4,
                'keypoint_confidence': 0.5,
                'hold_duration': 2.0,
                'drop_tolerance': 5,
            }],
            output='screen',
        ),

        # 2. 目标锁定：自动锁定最大人体，手势仅暂停
        Node(
            package='golf_navigation',
            executable='lock_manager',
            name='lock_manager',
            parameters=[{
                'lost_timeout': 10.0,
            }],
            output='screen',
        ),

        # 3. PID 跟随控制：锁定目标 + 深度图 → /cmd_vel_raw
        #    输出到 /cmd_vel_raw，经 collision_monitor 安全过滤后转发到 /cmd_vel
        Node(
            package='golf_navigation',
            executable='follow_target_publisher',
            name='follow_target_publisher',
            parameters=[{
                # === 户外跟随参数 ===
                'target_distance': 1000,    # mm（户外跟随距离 1m，防止一顿一顿）
                'max_speed': 0.5,           # m/s（户外适当提速）
                'angular_kp': 1.2,          # WheelTec 原版
                'angular_ki': 0.0,          # WheelTec 原版
                'angular_kd': 0.005,        # WheelTec 原版
                'linear_kp': 0.2,           # WheelTec 原版
                'linear_ki': 0.0,           # WheelTec 原版
                'linear_kd': 0.0,           # WheelTec 原版
                'out_of_range': 5000,       # mm（户外放宽到 5m）
                'min_depth': 300,           # mm
                'tan_half_fov': 1.7321,     # S11 ~120° FOV → tan(60°)
            }],
            output='screen',
        ),

        # 4. collision_monitor：/cmd_vel_raw → /cmd_vel（避障安全层）
        #    前方 0.5m 内障碍 → 停车，1.5m 内 → 减速 40%
        #    NOTE: 测试阶段暂不启用，手柄急停兜底。Phase 4 产品化时启用。
        #    启用时需同步将 follow_target_publisher 输出改回 /cmd_vel_raw
        # Node(
        #     package='nav2_collision_monitor',
        #     executable='collision_monitor',
        #     name='collision_monitor',
        #     parameters=[
        #         os.path.join(golf_bringup_dir, 'config', 'collision_monitor.yaml'),
        #     ],
        #     output='screen',
        # ),

        # 5. GPS 路点记录：跟随模式下自动记录 GPS 面包屑路点
        Node(
            package='golf_mapping',
            executable='gps_path_recorder',
            name='gps_path_recorder',
            output='screen',
        ),

        # 6. GPS 路点导航：PID模式，经过验证的参数（v8.2.4实车调优）
        Node(
            package='golf_navigation',
            executable='gps_waypoint_follower',
            name='gps_waypoint_follower',
            parameters=[{
                # v8.2.6 实车验证通过的参数（2026-04-07）
                'ang_kp': 0.2,               # 温柔转向（0.8太激进→蛇形）
                'max_angular': 0.3,           # 限制最大角速度（0.8→转圈）
                'bearing_dead_zone': 15.0,    # GPS噪声死区（5°→蛇形）
                'arrival_tolerance': 5.0,     # 到达判定
                'use_mppi': False,            # PID模式
                'use_pure_pursuit': False,    # 前瞻PID
            }],
            output='screen',
        ),

        # 7. 召唤服务：接收 APP 召唤请求 → 查路径图 → 发起导航
        Node(
            package='golf_navigation',
            executable='summon_service',
            name='summon_service',
            output='screen',
        ),

        # 8. MQTT 桥接：ROS2 ↔ MQTT 双向通信（4G 网络连接 APP）
        Node(
            package='golf_communication',
            executable='mqtt_bridge',
            name='mqtt_bridge',
            parameters=[{
                'mqtt_host': os.environ.get('MQTT_HOST', '144.202.105.250'),
                'mqtt_port': 1883,
                'vehicle_id': os.environ.get('VEHICLE_ID', 'cart_01'),
            }],
            output='screen',
        ),
    ])
