#!/bin/bash
# 高尔夫球车 — 全量 rosbag 录制脚本
# 用途：一次录制所有传感器+感知+控制+导航话题
# 用法：bash ~/golf_ws/scripts/record_all.sh

set -e

# 环境
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/wheeltec_ros2/install/setup.bash 2>/dev/null
source ~/golf_ws/install/setup.bash 2>/dev/null

# 输出目录
BAG_DIR=~/golf_ws/bags
mkdir -p "$BAG_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_NAME="full_bag_${TIMESTAMP}"
BAG_PATH="${BAG_DIR}/${BAG_NAME}"

# 全量话题（传感器+感知+控制+导航）
TOPICS=(
    # 传感器 — 相机
    /lx_camera_node/LxCamera_Rgb
    /lx_camera_node/LxCamera_Depth
    /lx_camera_node/LxCamera_TofInfo
    # 传感器 — LiDAR
    /scan
    # 传感器 — GPS
    /gps/fix
    /heading
    /heading_deg
    # 传感器 — IMU
    /h30_imu
    /h30_imu_raw
    # 传感器 — 底盘
    /odom_combined
    /joy
    # 感知
    /yolo/tracking
    /locked_target
    /gesture_cmd
    # 控制
    /cmd_vel
    /cmd_vel_nav
    /follow_state
    /system_mode
    /emergency_stop_state
    # 导航
    /nav_trigger
    /nav_complete
    /nav_blocked
    /path_graph/stats
    # TF
    /tf
    /tf_static
)

# Ctrl+C 处理
cleanup() {
    echo ""
    echo "==============================="
    echo " 录制结束"
    echo " 文件：${BAG_PATH}"
    echo "==============================="
    echo ""
    echo "验证命令："
    echo "  bash ~/golf_ws/scripts/verify_bag.sh ${BAG_PATH}"
    exit 0
}
trap cleanup SIGINT SIGTERM

# 打印说明
echo "==============================="
echo " 全量 rosbag 录制"
echo "==============================="
echo ""
echo "【录制话题】共 ${#TOPICS[@]} 个："
for t in "${TOPICS[@]}"; do
    echo "  $t"
done
echo ""
echo "输出路径：${BAG_PATH}"
echo "缓存上限：300MB"
echo ""
echo "按 Ctrl+C 停止录制"
echo "==============================="
echo ""
echo "3 秒后开始录制..."
sleep 3

ros2 bag record \
    --output "$BAG_PATH" \
    --max-cache-size $((300 * 1024 * 1024)) \
    "${TOPICS[@]}"
