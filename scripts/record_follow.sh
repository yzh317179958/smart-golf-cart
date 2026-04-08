#!/bin/bash
# 高尔夫球车 — 跟随场景 rosbag 录制脚本
# 用途：录制感知+跟随开发所需的全部话题
# 用法：bash ~/golf_ws/scripts/record_follow.sh

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
BAG_NAME="follow_bag_${TIMESTAMP}"
BAG_PATH="${BAG_DIR}/${BAG_NAME}"

# 录制话题列表
TOPICS=(
    /lx_camera_node/LxCamera_Rgb
    /lx_camera_node/LxCamera_Depth
    /lx_camera_node/LxCamera_TofInfo
    /yolo/tracking
    /scan
    /locked_target
    /follow_state
    /gesture_cmd
    /system_mode
    /cmd_vel
    /odom_combined
    /joy
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
echo " 跟随场景 rosbag 录制"
echo "==============================="
echo ""
echo "【录制前准备】"
echo "  1. 确保全栈已启动：bash ~/golf_ws/start_all.sh"
echo "  2. 确保 S11 相机正常（有 RGB + 深度数据）"
echo "  3. 确保 LiDAR 正常（有 /scan 数据）"
echo ""
echo "【录制步骤】"
echo "  1. 启动后，一人站在车前方 2-3 米处"
echo "  2. 缓慢行走 30-60 秒（直线+转弯），让车跟随"
echo "  3. 中途可做手势（停止/前进），测试手势识别"
echo "  4. 最后走出跟随范围（>5 米），测试丢失恢复"
echo "  5. 建议总时长 2-3 分钟"
echo ""
echo "【录制话题】共 ${#TOPICS[@]} 个："
for t in "${TOPICS[@]}"; do
    echo "  $t"
done
echo ""
echo "输出路径：${BAG_PATH}"
echo "缓存上限：200MB"
echo ""
echo "按 Ctrl+C 停止录制"
echo "==============================="
echo ""
echo "3 秒后开始录制..."
sleep 3

ros2 bag record \
    --output "$BAG_PATH" \
    --max-cache-size $((200 * 1024 * 1024)) \
    "${TOPICS[@]}"
