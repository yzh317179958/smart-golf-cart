"""跟随与导航 launch 文件

启动 8 个节点：模式管理 + 手势识别 + 目标锁定 + PID 跟随控制
  + GPS 路点记录 + GPS 路点脉冲跟随 + 召唤服务 + MQTT 桥接
依赖：sensors.launch.py + perception.launch.py 已启动
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    golf_bringup_dir = get_package_share_directory('golf_bringup')

    # 路点数据源隔离: production(实车实际运行) / test(回放或测试)
    # 默认 production; 测试时显式传 mode:=test 写入 data/test/path_graph.json
    data_file = PathJoinSubstitution([
        os.path.expanduser('~/golf_ws/data'),
        LaunchConfiguration('mode'),
        'path_graph.json',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='production',
            description='path_graph.json 数据源: production (实车) | test (回放/测试)',
        ),
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

        # 3. PID 跟随控制：锁定目标 + 深度图 → /cmd_vel（直发）
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

        # 4. GPS 路点记录：跟随模式下自动记录 GPS 面包屑路点
        Node(
            package='golf_mapping',
            executable='gps_path_recorder',
            name='gps_path_recorder',
            parameters=[{'data_file': data_file}],
            output='screen',
        ),

        # 5. GPS 路点导航：v8.3 脉冲转向
        Node(
            package='golf_navigation',
            executable='gps_waypoint_follower',
            name='gps_waypoint_follower',
            parameters=[{
                'max_speed': 0.6,             # 最大线速度
                'max_angular': 0.15,          # 安全上限角速度
                'bearing_dead_zone': 20.0,    # 脉冲触发死区
                'wall_threshold': 3.0,        # 侧边护栏触发距离
                'pulse_wall_w': 0.1,          # 避墙脉冲角速度
                'pulse_wall_on': 0.5,         # 避墙脉冲持续 (s)
                'pulse_nav_w': 0.12,          # 方向修正脉冲角速度
                'pulse_nav_off': 0.5,         # 方向冷却持续 (s)
                'arrival_tolerance': 5.0,     # 到达判定
                'cmd_vel_topic': '/cmd_vel',  # 直发底盘，不经中继
                'data_file': data_file,       # 按 mode 隔离 production/test
            }],
            output='screen',
        ),

        # 6. 召唤服务：接收 APP 召唤请求 → 查路径图 → 发起导航
        Node(
            package='golf_navigation',
            executable='summon_service',
            name='summon_service',
            parameters=[{'data_file': data_file}],
            output='screen',
        ),

        # 7. MQTT 桥接：ROS2 ↔ MQTT 双向通信（4G 网络连接 APP）
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
