#!/bin/bash
# 高尔夫球车 — rosbag 完整性验证脚本
# 用途：检查录制的 bag 文件是否包含关键话题
# 用法：bash ~/golf_ws/scripts/verify_bag.sh ~/golf_ws/bags/follow_bag_xxx

set -e

# 环境
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/wheeltec_ros2/install/setup.bash 2>/dev/null
source ~/golf_ws/install/setup.bash 2>/dev/null

# 参数检查
if [ -z "$1" ]; then
    echo "用法：bash verify_bag.sh <bag路径>"
    echo "示例：bash verify_bag.sh ~/golf_ws/bags/follow_bag_20260407_143000"
    exit 1
fi

BAG_PATH="$1"

if [ ! -d "$BAG_PATH" ]; then
    echo "[错误] bag 目录不存在：${BAG_PATH}"
    exit 1
fi

echo "==============================="
echo " Rosbag 完整性验证"
echo "==============================="
echo ""
echo "路径：${BAG_PATH}"
echo ""

# 获取 bag info
echo "--- bag 基本信息 ---"
BAG_INFO=$(ros2 bag info "$BAG_PATH" 2>&1)

if [ $? -ne 0 ]; then
    echo "[错误] 无法读取 bag 信息："
    echo "$BAG_INFO"
    exit 1
fi

echo "$BAG_INFO"
echo ""

# 提取时长
DURATION=$(echo "$BAG_INFO" | grep -i "duration" | head -1)
echo "时长：$DURATION"

# 提取消息总数
MSG_COUNT=$(echo "$BAG_INFO" | grep -i "message count" | head -1)
echo "消息：$MSG_COUNT"
echo ""

# 必需话题检查
REQUIRED_TOPICS=(
    /scan
    /system_mode
    /odom_combined
    /tf
)

echo "--- 关键话题检查 ---"
PASS=true

for topic in "${REQUIRED_TOPICS[@]}"; do
    if echo "$BAG_INFO" | grep -q "Topic: ${topic} "; then
        COUNT=$(echo "$BAG_INFO" | grep "Topic: ${topic} " | grep -oP 'Count:\s*\K[0-9]+' || echo "?")
        echo "  [通过] ${topic}  (${COUNT} 条消息)"
    elif echo "$BAG_INFO" | grep -q "${topic}"; then
        # 不同 ROS2 版本 bag info 格式可能不同，宽松匹配
        echo "  [通过] ${topic}  (已找到)"
    else
        echo "  [缺失] ${topic}"
        PASS=false
    fi
done

echo ""

# 额外话题统计（非必需，仅展示）
echo "--- 全部话题统计 ---"
TOPIC_COUNT=$(echo "$BAG_INFO" | grep -c "Topic:" || echo "0")
echo "  话题总数：${TOPIC_COUNT}"
echo ""

# 最终结果
echo "==============================="
if [ "$PASS" = true ]; then
    echo "  验证结果：通过"
    echo "  所有关键话题均存在"
else
    echo "  验证结果：未通过"
    echo "  缺少关键话题，请检查录制时全栈是否正常运行"
fi
echo "==============================="

if [ "$PASS" = false ]; then
    exit 1
fi
