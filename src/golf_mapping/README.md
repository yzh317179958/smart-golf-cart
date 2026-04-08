# golf_mapping

高尔夫球车地图与传感器融合包 -- GPS 面包屑路点记录、IMU 坐标系转换与双天线航向融合、导航测试数据日志。

## 节点列表

| 节点名 | 功能 | 订阅话题 | 发布话题 |
|--------|------|----------|----------|
| `gps_path_recorder` | 跟随过程中自动记录 GPS 面包屑路点，支持多次经过加权平均、跨 Session 帧对齐、路点标记 | `/gps/fix` (NavSatFix), `/gps/filtered` (NavSatFix), `/follow_state` (String), `/mark_waypoint_label` (String), `/odom_combined` (Odometry) | `/path_graph/stats` (String) |
| `imu_ned_to_enu` | H30 IMU NED→ENU 坐标转换 + G90 双天线航向融合，输出 EKF 可用的 ENU 坐标系 IMU 数据 | `/heading` (QuaternionStamped), `/h30_imu_raw` (Imu) | `/h30_imu` (Imu) |
| `test_logger` | 实车测试数据后台记录器，采集 GPS 轨迹 + 系统事件保存为 JSON 文件 | `/gps/fix` (NavSatFix), `/heading_deg` (Float32), `/cmd_vel` (Twist), `/system_mode` (String), `/follow_state` (String), `/nav_trigger` (String), `/nav_complete` (String), `/emergency_stop_state` (String) | 无 (写文件) |

## 参数说明

### gps_path_recorder

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `min_record_distance` | float | 3.0 | 相邻路点最小记录距离 (m, odom 判定) |
| `merge_distance` | float | 3.0 | 路点合并半径 (m, GPS haversine 判定) |
| `save_interval` | float | 300.0 | 自动保存间隔 (秒) |
| `min_speed` | float | 0.05 | 最小记录速度 (m/s, 防静态 GPS 漂移) |
| `data_file` | string | `~/golf_ws/data/path_graph.json` | 路径图 JSON 输出路径 |
| `coordinate_source` | string | `raw` | GPS 坐标源: `raw` = /gps/fix 裸 GPS, `ekf` = /gps/filtered EKF 平滑 |
| `align_distance` | float | 8.0 | 跨 Session 帧对齐检测距离 (m) |

### imu_ned_to_enu

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `yaw_offset` | float | -1.988 | H30 NED→ENU 偏移量 (rad, 含安装偏差 + 磁偏角, 仅回退时使用) |

### test_logger

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `data_dir` | string | `~/golf_ws/data` | 日志文件输出目录 |

## 编译方法

```bash
cd ~/golf_ws
colcon build --packages-select golf_mapping --symlink-install
source install/setup.bash
```

## 测试方法

### 单节点启动验证

```bash
# GPS 路点记录器（需要 GPS + TF + /follow_state 数据）
ros2 run golf_mapping gps_path_recorder
ros2 topic echo /path_graph/stats

# IMU 坐标转换（需要 H30 IMU 和 G90 航向数据）
ros2 run golf_mapping imu_ned_to_enu
ros2 topic hz /h30_imu

# 测试日志记录器（后台运行，Ctrl+C 停止后自动保存）
ros2 run golf_mapping test_logger
```

### 路点记录流程验证

```bash
# 1. 确认 GPS 有数据
ros2 topic hz /gps/fix

# 2. 确认 TF 有 odom_combined → base_footprint
ros2 run tf2_ros tf2_echo odom_combined base_footprint

# 3. 模拟跟随状态启动记录
ros2 topic pub /follow_state std_msgs/msg/String "data: 'tracking'" -r 10

# 4. 查看记录统计
ros2 topic echo /path_graph/stats

# 5. 标记路点
ros2 topic pub /mark_waypoint_label std_msgs/msg/String "data: '3号洞'" --once
```

### rosbag 回放测试

```bash
# 回放传感器数据
ros2 bag play sensor_bag/ --clock
ros2 run golf_mapping imu_ned_to_enu --ros-args -p use_sim_time:=true
ros2 run golf_mapping test_logger --ros-args -p use_sim_time:=true
```

### 测试日志可视化

```bash
# 测试完成后将日志从 WheelTec 拷贝到 VPS
scp wheeltec:~/golf_ws/data/test_log_*.json ./data/
# 用 generate_test_map.py 生成 HTML 地图
python3 scripts/generate_test_map.py data/test_log_YYYYMMDD_HHMMSS.json
```

## 注意事项

1. **路点坐标源**：默认使用裸 GPS (`coordinate_source=raw`)，路点精度依赖多次经过加权平均提升。切换到 `ekf` 模式需要 robot_localization navsat_transform 已正确运行。
2. **跨 Session 帧对齐**：GPS 系统性漂移 3-8m 会导致不同 session 的路点帧不一致。`gps_path_recorder` 在进入已有路点区域时自动检测帧差并回溯校正本 session 的路点，原理类似 SLAM 回环校正。
3. **距离判断与坐标记录分离**：相邻路点间距用 TF odom 欧式距离判断（里程计平滑，保证 3m 均匀间隔），路点坐标用 GPS 记录（全局一致）。
4. **G90 航向优先**：`imu_ned_to_enu` 优先使用 G90 双天线罗盘航向（精度 ±0.3 度），仅当 G90 超过 2 秒未更新时回退到 H30 磁力计。G90 航向的 orientation_covariance 比 H30 低两个数量级，EKF 会强信任 G90。
5. **FRD→FLU 变换**：`imu_ned_to_enu` 同时将 H30 的角速度和线加速度从 FRD (前右下) 转换为 FLU (前左上) 坐标系。
6. **测试日志采样率**：GPS 轨迹 5Hz 采样，系统事件仅记录状态变化（去重），每 60 秒自动保存一次。
7. **路径图文件格式**：`path_graph.json` 包含 `waypoints` (路点字典)、`edges` (双向边列表)、`wp_counter` (自增计数器)。
