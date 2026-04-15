#!/bin/bash
# 高尔夫球车全栈一键启动脚本（v8.2 GPS-PID 架构）
# 启动顺序：sensors → perception → follow（含导航+MQTT）
# 用法: bash ~/golf_ws/start_all.sh
#
# v8.2 架构：GPS-PID 导航 + MPPI 仅跟随避障
# 不再启动旧 gps_navigation.launch.py（Nav2 全栈）

set -e

# 环境
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/wheeltec_ros2/install/setup.bash
source ~/golf_ws/install/setup.bash 2>/dev/null

echo "==============================="
echo " Golf Cart v8.2 启动"
echo "==============================="

# 清理旧进程
pkill -9 -f 'golf_\|yolo\|camera_info_fix\|lx_camera\|nmea_serial\|lslidar\|lidar_emergency\|summon_service\|mqtt_bridge\|imu_ned_to_enu\|yesense_node\|ekf_gps\|navsat_transform\|mode_manager\|lock_manager\|gesture_node\|gps_path_recorder\|gps_waypoint_follower\|follow_target_publisher' 2>/dev/null || true
tmux kill-server 2>/dev/null || true
rm -rf /dev/shm/fastrtps_* /dev/shm/fast_datasharing* 2>/dev/null || true
rm -rf ~/.ros/log/* 2>/dev/null || true
mkdir -p ~/golf_ws/logs
sleep 2

# Step 1: sensors（底盘+LiDAR+S11+GPS+IMU+EKF+foxglove）
echo "[1/3] 启动 sensors..."
tmux new-session -d -s golf -n sensors \
  "source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && source ~/wheeltec_ros2/install/setup.bash && source ~/golf_ws/install/setup.bash 2>/dev/null && ros2 launch golf_bringup sensors.launch.py 2>&1 | tee ~/golf_ws/logs/sensors.log"
sleep 15

# 检查 GPS
if timeout 3 ros2 topic echo /gps/fix --once 2>/dev/null | grep -q "latitude"; then
    echo "  [ok] GPS 正常"
else
    echo "  [!] GPS 无数据（室内正常）"
fi

# Step 2: perception（YOLO 检测+跟踪）
echo "[2/3] 启动 perception..."
tmux new-window -t golf -n perception \
  "source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && source ~/wheeltec_ros2/install/setup.bash && source ~/golf_ws/install/setup.bash 2>/dev/null && ros2 launch golf_bringup perception.launch.py 2>&1 | tee ~/golf_ws/logs/perception.log"
sleep 15

if timeout 5 ros2 topic hz /yolo/tracking 2>/dev/null | grep -q "average rate"; then
    echo "  [ok] YOLO 感知正常"
else
    echo "  [!] YOLO 等待图像输入（S11 未连接时正常）"
fi

# Step 3: follow（模式管理+PID跟随+GPS路点记录+GPS-PID导航+LiDAR急停+召唤+MQTT）
echo "[3/3] 启动 follow..."
tmux new-window -t golf -n follow \
  "source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && source ~/wheeltec_ros2/install/setup.bash && source ~/golf_ws/install/setup.bash 2>/dev/null && ros2 launch golf_bringup follow.launch.py 2>&1 | tee ~/golf_ws/logs/follow.log"
sleep 8

# 验证关键节点
echo ""
echo "=== 验证关键节点 ==="
for node in mode_manager gps_waypoint_follower mqtt_bridge gps_path_recorder; do
    if ros2 node list 2>/dev/null | grep -q "$node"; then
        echo "  [ok] $node"
    else
        echo "  [!] $node 未启动"
    fi
done

echo ""
echo "==============================="
echo " 全栈启动完成！"
echo "==============================="
echo " 手机访问: http://144.202.105.250:8080"
echo " tmux 查看: tmux attach -t golf"
echo " 停止: bash ~/golf_ws/stop_all.sh"
echo "==============================="
