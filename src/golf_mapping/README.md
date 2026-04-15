# golf_mapping

高尔夫球车地图与传感器融合包 -- GPS 面包屑路点记录、IMU 坐标系转换与双天线航向融合、导航测试数据日志。

## 模块职责

本包是整套系统的**地图数据生产部门**：接收 GPS/IMU 原始传感器信息，加工成给规控层消费的路点图文件，并产出可视化验收产物。

- **输入**：`/gps/fix` 裸 GPS + `/heading` G90 双天线航向 + `/h30_imu_raw` IMU
- **输出**：`data/{production,test}/path_graph.json` + 卫星地图 HTML 可视化
- **下游**：`golf_navigation` 的 `gps_waypoint_follower` 和 `summon_service` 读取 `path_graph.json`
- **责任人**：GPS / 地图传感器工程师（独立负责本包全部 3 个节点）
- **禁区**：不得修改 `golf_navigation/` 和 `golf_perception/` 任何文件；跨包通信严格走话题契约，见 `docs/INTERFACE_CONTRACT.md`

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
| `data_file` | string | `~/golf_ws/data/production/path_graph.json` | 路径图 JSON 输出路径（默认写生产目录；测试请用 `mode:=test` launch 参数切到 `data/test/`） |
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

三层测试体系：L1 本地纯回放 → L2 远程 WheelTec bag 回放 → L3 实车户外（仅负责人执行）。工程师应能独立走完 L1 + L2，L3 提交代码后由负责人现场验证。

### L1 本地 bag 回放（engineer 日常开发）

从负责人处拿到 `nav_bag_*.tar.gz`（含 bag 和基线 path_graph.json）后，在本地 ROS2 Humble 环境完成：

```bash
# 解压后只读基线（绝不覆盖生产数据）
mkdir -p ~/golf_ws/data/test
cp nav_bag_*/baseline_path_graph.json ~/golf_ws/data/test/path_graph.json.baseline

# 回放 bag
ros2 bag play nav_bag_*/ --clock --loop

# 另开终端：用 test 模式启动 recorder（写入 data/test/，不污染 production）
ros2 run golf_mapping gps_path_recorder \
    --ros-args -p use_sim_time:=true \
    -p data_file:=$HOME/golf_ws/data/test/session_1.json

# 观察记录统计
ros2 topic echo /path_graph/stats

# IMU 坐标转换验证
ros2 run golf_mapping imu_ned_to_enu --ros-args -p use_sim_time:=true
ros2 topic hz /h30_imu
```

### L2 远程 WheelTec bag 回放（engineer 需 SSH 权限）

通过 VPS 跳板 SSH 到 WheelTec 上 rsync 代码、编译、回放 bag 验证新代码在目标硬件上编译运行正常。**禁止触碰实车驾驶和 `data/production/` 目录**。

```bash
# 代码同步到 WheelTec（排除 build/install）
rsync -avz --exclude='build/' --exclude='install/' --exclude='third_party/lx_camera_ros/' \
    ~/golf_ws/src/golf_mapping/ wheeltec:~/golf_ws/src/golf_mapping/

# WheelTec 上编译
ssh wheeltec 'cd ~/golf_ws && colcon build --packages-select golf_mapping --symlink-install'

# WheelTec 上回放 bag + 跑 test 模式 recorder
ssh wheeltec 'bash -lc "source ~/golf_ws/install/setup.bash && \
    ros2 bag play ~/golf_ws/bags/nav_bag_YYYYMMDD_HHMMSS/ --clock &
    ros2 run golf_mapping gps_path_recorder \
        --ros-args -p use_sim_time:=true \
        -p data_file:=\$HOME/golf_ws/data/test/session_1.json"'
```

### L3 实车户外测试（仅负责人执行）

engineer 通过 PR 提交改动，负责人 merge 后在车上执行：

```bash
# 负责人在 WheelTec 上启动完整系统（生产模式）
bash ~/golf_ws/start_all.sh

# 或手动启动仅 mapping 层做路点录制
ros2 launch golf_bringup follow.launch.py mode:=production
```

### 验收标准

- **路点准确性**：`path_graph.json` 中的路点在卫星地图上目测贴合实际道路，不压线、不进草地
- **Session 可重复性**：同一路段 3 次记录后叠加，互相偏差 ≤ GPS 自身 CEP (1.5m)
- **IMU 航向一致性**：G90 可用时 `/h30_imu` 的 yaw 和 G90 `/heading_deg` 偏差 ≤ 1°；G90 丢失 ≥ 2s 时回退到 H30 行为正常
- **生产数据保护**：整个测试过程 `data/production/path_graph.json` 的文件修改时间戳**不变**

## 交付物

engineer 本地或 L2 测试完成后，在 PR 中提供以下产物：

| 文件 | 位置 | 用途 |
|------|------|------|
| `session_{1,2,3}.json` | `data/test/` | 沙箱路点图（多次回放或多次实车跑同一路段的结果） |
| `waypoints_overlay_3session.html` | `data/test/` | 三 session 叠加在卫星底图的可视化（用 `scripts/generate_waypoint_map.py` 生成） |
| PR 描述 | GitHub | "改了什么 / 为什么改 / 效果对比（附前后 HTML 截图）" |

**禁止物**：任何对 `data/production/` 的写操作、任何对 `golf_navigation/` 或 `golf_perception/` 的修改、任何对节点话题接口的改动。

## 注意事项

0. **生产/测试数据隔离铁律**：`data_file` 默认指向 `data/production/path_graph.json`，严禁测试过程直接写入此路径。测试必须通过 `ros2 launch golf_bringup follow.launch.py mode:=test` 或显式传 `-p data_file:=~/golf_ws/data/test/xxx.json` 隔离到 `data/test/` 目录。生产数据是实车运行的唯一真实来源，一旦污染会导致召唤和导航指到错误坐标。
1. **路点坐标源**：默认使用裸 GPS (`coordinate_source=raw`)，路点精度依赖多次经过加权平均提升。切换到 `ekf` 模式需要 robot_localization navsat_transform 已正确运行。
2. **跨 Session 帧对齐**：GPS 系统性漂移 3-8m 会导致不同 session 的路点帧不一致。`gps_path_recorder` 在进入已有路点区域时自动检测帧差并回溯校正本 session 的路点，原理类似 SLAM 回环校正。
3. **距离判断与坐标记录分离**：相邻路点间距用 TF odom 欧式距离判断（里程计平滑，保证 3m 均匀间隔），路点坐标用 GPS 记录（全局一致）。
4. **G90 航向优先**：`imu_ned_to_enu` 优先使用 G90 双天线罗盘航向（精度 ±0.3 度），仅当 G90 超过 2 秒未更新时回退到 H30 磁力计。G90 航向的 orientation_covariance 比 H30 低两个数量级，EKF 会强信任 G90。
5. **FRD→FLU 变换**：`imu_ned_to_enu` 同时将 H30 的角速度和线加速度从 FRD (前右下) 转换为 FLU (前左上) 坐标系。
6. **测试日志采样率**：GPS 轨迹 5Hz 采样，系统事件仅记录状态变化（去重），每 60 秒自动保存一次。
7. **路径图文件格式**：`path_graph.json` 包含 `waypoints` (路点字典)、`edges` (双向边列表)、`wp_counter` (自增计数器)。
