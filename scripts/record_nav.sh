#!/bin/bash
# 高尔夫球车 — 导航场景 rosbag 录制脚本
# 用途：录制导航+建图开发所需的全部话题
# 用法：bash ~/golf_ws/scripts/record_nav.sh

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
BAG_NAME="nav_bag_${TIMESTAMP}"
BAG_PATH="${BAG_DIR}/${BAG_NAME}"

# 录制话题列表
TOPICS=(
    /gps/fix
    /heading_deg
    /heading
    /h30_imu
    /h30_imu_raw
    /scan
    /cmd_vel
    /cmd_vel_nav
    /nav_trigger
    /nav_complete
    /nav_blocked
    /emergency_stop_state
    /system_mode
    /odom_combined
    /follow_state
    /path_graph/stats
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
echo " 导航场景 rosbag 录制"
echo "==============================="
echo ""
echo "【录制前准备】"
echo "  1. 确保全栈已启动：bash ~/golf_ws/start_all.sh"
echo "  2. 确保 GPS 已定位（/gps/fix 有数据，status >= 0）"
echo "  3. 确保路点已录制（path_graph.json 存在）"
echo "  4. 确保 LiDAR 正常（有 /scan 数据）"
echo ""
echo "【录制步骤】"
echo "  1. 启动后，通过 Web APP 或 MQTT 发送导航目标"
echo "  2. 观察车辆自动行驶，直到到达目标或中途停止"
echo "  3. 可测试多段导航（不同路点间来回）"
echo "  4. 可测试障碍物阻挡场景（人挡在路中间）"
echo "  5. 建议总时长 3-5 分钟（含多段导航）"
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
