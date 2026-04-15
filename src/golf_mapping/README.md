# golf_mapping

高尔夫球车地图与传感器融合包：GPS 面包屑路点记录、H30 IMU NED→ENU 坐标转换、G90 双天线航向融合、实车测试数据日志。

## 节点列表

| 节点名 | 功能 | 订阅话题 | 发布话题 / 输出 |
|--------|------|----------|-----------------|
| `gps_path_recorder` | 跟随过程中自动记录 GPS 面包屑路点，支持多次经过加权平均、跨 Session 帧对齐、路点标记 | `/gps/fix`, `/gps/filtered`, `/follow_state`, `/mark_waypoint_label`, `/odom_combined` | `/path_graph/stats`；写入 `path_graph.json` |
| `imu_ned_to_enu` | H30 IMU NED→ENU 坐标转换 + G90 双天线航向优先融合，输出 EKF 可用的 ENU 坐标系 IMU | `/heading`, `/h30_imu_raw` | `/h30_imu` |
| `test_logger` | 实车测试数据后台记录器，采集 GPS 轨迹 + 系统事件 | `/gps/fix`, `/heading_deg`, `/cmd_vel`, `/system_mode`, `/follow_state`, `/nav_trigger`, `/nav_complete`, `/emergency_stop_state` | 写入 `test_log_*.json` |

## 参数说明

### gps_path_recorder

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `min_record_distance` | float | 3.0 | 相邻路点最小记录距离 (m, odom 判定) |
| `merge_distance` | float | 3.0 | 路点合并半径 (m, GPS haversine 判定) |
| `save_interval` | float | 300.0 | 自动保存间隔 (秒) |
| `min_speed` | float | 0.05 | 最小记录速度 (m/s, 防静态 GPS 漂移) |
| `data_file` | string | `~/golf_ws/data/production/path_graph.json` | 路径图 JSON 输出路径（由 `mode` launch 参数切换 production/test） |
| `coordinate_source` | string | `raw` | GPS 坐标源：`raw` = `/gps/fix` 裸 GPS，`ekf` = `/gps/filtered` 平滑 |
| `align_distance` | float | 8.0 | 跨 Session 帧对齐检测距离 (m) |

### imu_ned_to_enu

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `yaw_offset` | float | -1.988 | H30 NED→ENU 偏移量 (rad，含安装偏差 + 磁偏角，仅回退时使用) |

### test_logger

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `data_dir` | string | `~/golf_ws/data` | 日志文件输出目录 |

## 上下游接口

- **上游**：
  - `wheeltec_gps_driver` 提供 `/gps/fix` (NavSatFix) 和 `/heading` (QuaternionStamped)
  - H30 IMU 驱动提供 `/h30_imu_raw` (Imu)
  - `robot_localization/ekf_odom` 提供 `/odom_combined` (Odometry)
- **下游**：
  - `golf_navigation/gps_waypoint_follower` 读取 `path_graph.json` 做 Dijkstra 规划
  - `golf_navigation/summon_service` 读取 `path_graph.json` 查球洞名对应坐标
  - `robot_localization/ekf_gps` 订阅 `/h30_imu` 做航向融合

## 编译

```bash
cd ~/golf_ws
colcon build --packages-select golf_mapping --symlink-install
source install/setup.bash
```

## 启动

通过 `golf_bringup` 统一 launch（推荐）：

```bash
# production（实车，默认写 data/production/path_graph.json）
ros2 launch golf_bringup follow.launch.py

# test（回放/测试，写 data/test/path_graph.json）
ros2 launch golf_bringup follow.launch.py mode:=test
```

单节点启动：

```bash
ros2 run golf_mapping gps_path_recorder
ros2 run golf_mapping imu_ned_to_enu
ros2 run golf_mapping test_logger
```

## 注意事项

1. **生产/测试数据隔离**：`data_file` 默认指向 `data/production/path_graph.json`，测试必须通过 `mode:=test` 或显式 `-p data_file:=~/golf_ws/data/test/xxx.json` 隔离，防止污染实车路点。
2. **G90 航向优先**：`imu_ned_to_enu` 优先使用 G90 双天线罗盘航向 (±0.3°)，G90 超过 2 秒无更新时回退到 H30 磁力计。
3. **路径图文件格式**：`path_graph.json` 包含 `waypoints`（ID → lat/lon/label）、`edges`（双向边 + 距离）、`wp_counter`（自增计数器）。
