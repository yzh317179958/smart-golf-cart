#!/bin/bash
# 纯传感器启动脚本（不启动感知/跟随/导航）
# 用途：录制传感器原始数据、调试传感器
# 用法: bash ~/golf_ws/start_sensors.sh
#
# 启动内容：WheelTec底盘 + N10P LiDAR + S11摄像头 + GPS + H30 IMU + GPS EKF

set -e

# 环境
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/wheeltec_ros2/install/setup.bash
source ~/golf_ws/install/setup.bash 2>/dev/null

echo "==============================="
echo " Sensors Only 启动"
echo "==============================="

# 清理旧 sensor 进程（不动底盘驱动）
pkill -9 -f 'camera_info_fix\|lx_camera\|nmea_serial\|lslidar\|imu_ned_to_enu\|yesense_node\|ekf_gps\|navsat_transform\|depthimage_to_laserscan' 2>/dev/null || true
tmux kill-session -t sensors 2>/dev/null || true
rm -rf /dev/shm/fastrtps_* /dev/shm/fast_datasharing* 2>/dev/null || true
rm -rf ~/.ros/log/* 2>/dev/null || true
sleep 2

# 启动 sensors.launch.py
echo "[1/1] 启动 sensors..."
tmux new-session -d -s sensors -n sensors \
  "source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && source ~/wheeltec_ros2/install/setup.bash && source ~/golf_ws/install/setup.bash 2>/dev/null && ros2 launch golf_bringup sensors.launch.py 2>&1"
sleep 15

# 验证关键话题
echo ""
echo "=== 验证传感器 ==="

if timeout 3 ros2 topic echo /scan --once 2>/dev/null | grep -q "ranges"; then
    echo "  [ok] LiDAR /scan"
else
    echo "  [!] LiDAR 无数据"
fi

if timeout 3 ros2 topic echo /gps/fix --once 2>/dev/null | grep -q "latitude"; then
    echo "  [ok] GPS /gps/fix"
else
    echo "  [!] GPS 无数据（室内正常）"
fi

if timeout 3 ros2 topic echo /h30_imu --once 2>/dev/null | grep -q "orientation"; then
    echo "  [ok] IMU /h30_imu"
else
    echo "  [!] IMU 无数据"
fi

if timeout 3 ros2 topic echo /odom_combined --once 2>/dev/null | grep -q "pose"; then
    echo "  [ok] 底盘 /odom_combined"
else
    echo "  [!] 底盘 odom 无数据"
fi

echo ""
echo "==============================="
echo " 传感器启动完成！"
echo "==============================="
echo " tmux 查看: tmux attach -t sensors"
echo " 录制: bash ~/golf_ws/record_all.sh"
echo " 停止: bash ~/golf_ws/stop_all.sh"
echo "==============================="
