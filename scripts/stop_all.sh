#!/bin/bash
# 安全停车 + 关闭所有 golf ROS2 节点
# 用法: ssh wheeltec-4g "bash ~/golf_ws/stop_all.sh"
#
# 重要：不能用 killall python3，会漏杀或误杀。
# 必须按进程名精确匹配，用 SIGINT 让 ROS2 节点走析构流程。
# 否则 lidar_emergency_stop / follow_target_publisher 残留会抢占 /cmd_vel，导致手柄失控。

set -e

echo "=== 1. 发送零速停车 ==="
source /opt/ros/humble/setup.bash
source ~/wheeltec_ros2/install/setup.bash 2>/dev/null
source ~/golf_ws/install/setup.bash 2>/dev/null
timeout 5 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}' 2>/dev/null || true

echo "=== 2. SIGINT 优雅关闭 golf 节点 ==="
# follow 层（follow.launch.py 全部节点）
pkill -INT -f 'follow_target_publisher' 2>/dev/null || true
pkill -INT -f 'mode_manager' 2>/dev/null || true
pkill -INT -f 'lock_manager' 2>/dev/null || true
pkill -INT -f 'gesture_node' 2>/dev/null || true
pkill -INT -f 'gps_path_recorder' 2>/dev/null || true
pkill -INT -f 'gps_waypoint_follower' 2>/dev/null || true
pkill -INT -f 'lidar_emergency_stop' 2>/dev/null || true
pkill -INT -f 'summon_service' 2>/dev/null || true
pkill -INT -f 'mqtt_bridge' 2>/dev/null || true

# gps_pid_nav 层（gps_pid_nav.launch.py）
pkill -INT -f 'test_logger' 2>/dev/null || true
pkill -INT -f 'relay.*cmd_vel_nav' 2>/dev/null || true

# gps_navigation 层（Nav2 C++ 节点，用 killall 按可执行文件名杀）
killall -9 lifecycle_manager controller_server planner_server bt_navigator waypoint_follower 2>/dev/null || true

# perception 层
pkill -INT -f 'yolo_node' 2>/dev/null || true
pkill -INT -f 'tracking_node' 2>/dev/null || true

# sensors 层
pkill -INT -f 'camera_info_fix' 2>/dev/null || true
pkill -INT -f 'depthimage_to_laserscan' 2>/dev/null || true
pkill -INT -f 'nmea_serial_driver' 2>/dev/null || true
pkill -INT -f 'foxglove_bridge' 2>/dev/null || true
pkill -INT -f 'lx_camera_node' 2>/dev/null || true
pkill -INT -f 'lslidar_driver' 2>/dev/null || true
pkill -INT -f 'imu_ned_to_enu' 2>/dev/null || true
pkill -INT -f 'yesense_node_publisher' 2>/dev/null || true
pkill -INT -f 'ekf_gps_node' 2>/dev/null || true
pkill -INT -f 'navsat_transform_node' 2>/dev/null || true

sleep 3

echo "=== 3. 杀 tmux golf 会话 ==="
tmux kill-session -t golf 2>/dev/null || true
tmux kill-session -t sensors 2>/dev/null || true
tmux kill-session -t perception 2>/dev/null || true
tmux kill-session -t follow 2>/dev/null || true
tmux kill-session -t gps_nav 2>/dev/null || true
tmux kill-session -t gps_pid_nav 2>/dev/null || true

echo "=== 4. 检查残留进程 ==="
REMAINING=$(pgrep -af 'golf_\|yolo\|camera_info_fix\|foxglove\|lx_camera\|nmea_serial\|lslidar\|controller_server\|planner_server\|bt_navigator\|waypoint_follower\|lifecycle_manager\|lidar_emergency\|summon_service\|mqtt_bridge\|imu_ned_to_enu\|test_logger\|relay.*cmd_vel' 2>/dev/null | grep -v grep || true)
if [ -n "$REMAINING" ]; then
    echo "发现残留进程，发送 SIGKILL:"
    echo "$REMAINING"
    pkill -9 -f 'golf_\|yolo\|camera_info_fix\|foxglove\|lx_camera\|nmea_serial\|lslidar\|controller_server\|planner_server\|bt_navigator\|waypoint_follower\|lifecycle_manager\|lidar_emergency\|summon_service\|mqtt_bridge\|imu_ned_to_enu\|test_logger' 2>/dev/null || true
    sleep 2
fi

echo "=== 5. 确认底盘驱动仍在运行 ==="
if pgrep -f 'wheeltec_robot_node' > /dev/null 2>&1; then
    echo "底盘驱动正常运行（手柄可用）"
else
    echo "警告：底盘驱动未运行！手柄可能无法使用"
fi

echo "=== 停车完成 ==="
