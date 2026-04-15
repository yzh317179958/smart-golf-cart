#!/bin/bash
# GPS 路点记录最小测试集
#
# 用法（在 WheelTec 上运行）：
#   bash ~/golf_ws/scripts/test_waypoint.sh
#
# 测试流程：
#   1. 本脚本启动 sensors + GPS驱动 + gps_path_recorder + 模拟tracking状态
#   2. 用手柄遥控车辆走直线
#   3. VPS 上拷贝路点并生成离线地图：
#      scp -P 2225 wheeltec@localhost:~/golf_ws/data/path_graph.json ~/golf_ws/data/
#      python3 ~/golf_ws/scripts/generate_waypoint_map.py
#   4. 传到笔记本查看：scp ~/golf_ws/data/waypoints_latest.html laptop:~/Desktop/
#
# 注意：不要启动 follow.launch.py（会抢 cmd_vel + 覆盖 follow_state）

set -e

echo "=== GPS 路点记录测试 ==="
echo "1. 清理旧进程..."
sudo pkill -9 -f ros2 2>/dev/null || true
sudo pkill -9 -f 'controller_server|planner_server|bt_navigator' 2>/dev/null || true
sleep 2

echo "2. 清理旧日志..."
rm -rf ~/.ros/log/* 2>/dev/null || true

echo "3. source 环境..."
source /opt/ros/humble/setup.bash
source ~/wheeltec_ros2/install/setup.bash
source ~/golf_ws/install/setup.bash

echo "4. 启动 sensors.launch.py（底盘+LiDAR+GPS+EKF+手柄）..."
ros2 launch golf_bringup sensors.launch.py &
SENSORS_PID=$!
sleep 10

echo "5. 启动 gps_path_recorder..."
ros2 run golf_control gps_path_recorder &
RECORDER_PID=$!
sleep 2

echo "6. 持续发布 follow_state=tracking（模拟跟随状态）..."
ros2 topic pub /follow_state std_msgs/String "data: tracking" -r 1 &
PUB_PID=$!

echo ""
echo "=== 测试就绪 ==="
echo "用手柄遥控车辆走直线，观察路点记录日志"
echo "跑完后在 VPS 生成离线地图: python3 ~/golf_ws/scripts/generate_waypoint_map.py"
echo ""
echo "按 Ctrl+C 停止所有进程"

# 等待中断
trap "echo '停止中...'; kill $PUB_PID $RECORDER_PID $SENSORS_PID 2>/dev/null; sudo pkill -9 -f ros2 2>/dev/null; echo '已停止'" INT
wait
