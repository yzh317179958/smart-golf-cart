#!/bin/bash
# 部署跟随功能到 WheelTec（经笔记本跳板）
# 用法: bash scripts/deploy_follow.sh

set -e

REMOTE="wheeltec@192.168.0.100"
PROXY="-o ProxyJump=laptop"
REMOTE_WS="/home/wheeltec/golf_ws"

echo "=== 同步跟随功能代码 ==="

# 1. golf_perception (gesture_node, camera_info_fix)
echo "[1/3] 同步 golf_perception..."
scp $PROXY -r src/golf_perception/golf_perception/gesture_node.py \
    ${REMOTE}:${REMOTE_WS}/src/golf_perception/golf_perception/
scp $PROXY -r src/golf_perception/setup.py \
    ${REMOTE}:${REMOTE_WS}/src/golf_perception/

# 2. golf_control (lock_manager, follow_target_publisher)
echo "[2/3] 同步 golf_control..."
scp $PROXY -r src/golf_control/ ${REMOTE}:${REMOTE_WS}/src/

# 3. golf_bringup (follow.launch.py)
echo "[3/3] 同步 follow.launch.py..."
scp $PROXY src/golf_bringup/launch/follow.launch.py \
    ${REMOTE}:${REMOTE_WS}/src/golf_bringup/launch/

echo ""
echo "=== 远程编译 ==="
ssh $PROXY ${REMOTE} "cd ${REMOTE_WS} && source /opt/ros/humble/setup.bash && source ~/wheeltec_ros2/install/setup.bash && colcon build --symlink-install --packages-select golf_perception golf_control golf_bringup"

echo ""
echo "=== 部署完成 ==="
echo "测试启动顺序:"
echo "  终端1: ros2 launch golf_bringup sensors.launch.py"
echo "  终端2: ros2 launch golf_bringup navigation.launch.py"
echo "  终端3: ros2 launch golf_bringup perception.launch.py"
echo "  终端4: ros2 launch golf_bringup follow.launch.py"
