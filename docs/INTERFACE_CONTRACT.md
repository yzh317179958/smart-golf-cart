# ROS2 接口契约文档

> **项目**：智能高尔夫球车自动驾驶系统  
> **版本**：v8.2.7  
> **日期**：2026-04-07  
> **状态**：生效中

---

## 1. 文档目的

本文档定义了高尔夫球车 ROS2 系统中**模块间的接口契约**——即各功能模块之间通过 ROS2 话题和共享文件进行通信的"API 边界"。

**核心原则**：

- 每个话题的消息类型、QoS 策略、发布频率、发布方和订阅方构成一份**不可单方面变更的契约**。
- 任何工程师修改话题定义，必须事先通知所有受影响的模块负责人并取得一致同意。
- 本文档是模块集成的唯一权威参考，代码实现必须与本文档保持一致。

---

## 2. 话题契约表

### 2.1 感知层（PERCEPTION）

感知层负责目标检测、手势识别、目标锁定与跟随状态管理。

| 话题 | 消息类型 | QoS | 频率(Hz) | 发布方 | 订阅方 |
|------|----------|-----|----------|--------|--------|
| `/yolo/tracking` | `yolo_msgs/DetectionArray` | default | 15 | yolo_ros | gesture_node, lock_manager |
| `/gesture_cmd` | `std_msgs/String` | default | 事件触发 | gesture_node | mode_manager |
| `/locked_target` | `yolo_msgs/Detection` | default | 15 | lock_manager | follow_target_publisher |
| `/follow_state` | `std_msgs/String` | default | 事件触发 | lock_manager | mode_manager, mqtt_bridge, gps_path_recorder, follow_target_publisher |
| `/detections` | `vision_msgs/Detection2DArray` | default | 15 | detection_node | （调试用） |
| `/detection_keypoints` | `std_msgs/String` | default | 15 | detection_node | （调试用） |

**字段说明**：

- `/gesture_cmd`：手势命令字符串，如 `"stop"`、`"pause"`、`"follow"` 等。
- `/follow_state`：跟随状态字符串，如 `"TRACKING"`、`"LOST"`、`"IDLE"` 等。
- `/detections` 和 `/detection_keypoints` 仅用于调试可视化，不作为生产链路依赖。

---

### 2.2 控制层（CONTROL）

控制层负责系统模式管理、GPS 导航、速度指令下发与安全急停。

| 话题 | 消息类型 | QoS | 频率(Hz) | 发布方 | 订阅方 |
|------|----------|-----|----------|--------|--------|
| `/system_mode` | `std_msgs/String` | default | 10 | mode_manager | follow_target_publisher, gps_waypoint_follower, mqtt_bridge |
| `/cmd_vel` | `geometry_msgs/Twist` | default | 变化 | follow_target_publisher 或 lidar_emergency_stop | 底盘驱动 |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | default | 10 | gps_waypoint_follower | lidar_emergency_stop |
| `/nav_trigger` | `std_msgs/String` | QoS(depth=1) | 事件触发 | mqtt_bridge, summon_service | mode_manager, gps_waypoint_follower |
| `/nav_complete` | `std_msgs/String` | QoS(depth=1) | 事件触发 | gps_waypoint_follower | mode_manager, summon_service, mqtt_bridge |
| `/nav_blocked` | `std_msgs/String` | default | 事件触发 | lidar_emergency_stop | gps_waypoint_follower |
| `/emergency_stop_state` | `std_msgs/String` | default | 事件触发 | lidar_emergency_stop | （调试/日志） |

**字段说明**：

- `/system_mode`：系统模式字符串，如 `"IDLE"`、`"FOLLOW"`、`"NAV"` 等。
- `/cmd_vel`：最终下发到底盘的速度指令。在跟随模式下由 follow_target_publisher 直接发布；在导航模式下由 lidar_emergency_stop 中继 `/cmd_vel_nav` 并附加急停保护。
- `/cmd_vel_nav`：GPS 导航产生的原始速度指令，经过急停节点过滤后转发至 `/cmd_vel`。
- `/nav_trigger`：导航触发命令，包含目标路点信息。
- `/nav_complete`：导航完成通知。
- `/nav_blocked`：障碍物阻塞通知，导航节点收到后进入等待或重规划。

---

### 2.3 传感器层（SENSOR）

传感器层提供 GPS、IMU、LiDAR、相机等原始及融合数据。

| 话题 | 消息类型 | QoS | 频率(Hz) | 发布方 | 订阅方 |
|------|----------|-----|----------|--------|--------|
| `/gps/fix` | `sensor_msgs/NavSatFix` | BEST_EFFORT | 1-5 | GPS 驱动 | gps_path_recorder, gps_waypoint_follower, mqtt_bridge |
| `/gps/filtered` | `sensor_msgs/NavSatFix` | default | 1-5 | navsat_transform | gps_path_recorder（EKF 模式） |
| `/heading_deg` | `std_msgs/Float32` | default | 1-5 | GPS 驱动 | gps_waypoint_follower |
| `/heading` | `geometry_msgs/QuaternionStamped` | default | 1-5 | GPS 驱动 | imu_ned_to_enu |
| `/h30_imu_raw` | `sensor_msgs/Imu` | default | 200 | yesense 驱动 | imu_ned_to_enu |
| `/h30_imu` | `sensor_msgs/Imu` | default | 200 | imu_ned_to_enu | EKF |
| `/scan` | `sensor_msgs/LaserScan` | BEST_EFFORT | 12 | N10P 驱动 | lidar_emergency_stop, gps_waypoint_follower, costmap |
| `/odom_combined` | `nav_msgs/Odometry` | default | 20 | WheelTec EKF | gps_path_recorder |
| `/lx_camera_node/LxCamera_Rgb` | `sensor_msgs/Image` | default | 15 | S11 驱动 | detection_node（经 yolo_ros） |
| `/lx_camera_node/LxCamera_Depth` | `sensor_msgs/Image` | default | 15 | S11 驱动 | follow_target_publisher, camera_info_fix |
| `/joy` | `sensor_msgs/Joy` | default | 变化 | 手柄驱动 | mode_manager |

**字段说明**：

- `/gps/fix`：裸 GPS 定位数据，水平精度约 1.5m CEP。
- `/gps/filtered`：经 EKF 融合后的 GPS 数据，仅在 EKF 模式下使用。
- `/heading_deg`：双天线 GPS 绝对航向角（度），精度 ±0.3 度。
- `/heading`：双天线 GPS 航向四元数，供 IMU 融合节点使用。
- `/h30_imu_raw`：H30 IMU 原始数据（NED 坐标系）。
- `/h30_imu`：经 NED 转 ENU 并融合 GPS 航向后的 IMU 数据。
- `/scan`：N10P 激光雷达扫描数据，后方 150-210 度已屏蔽（避免 GPS 天线误检）。

---

### 2.4 通信层（COMMUNICATION）

通信层负责外部系统（Web APP、MQTT）与车载系统之间的消息桥接。

| 话题 | 消息类型 | QoS | 频率(Hz) | 发布方 | 订阅方 |
|------|----------|-----|----------|--------|--------|
| `/summon_request` | `std_msgs/String` | QoS(depth=1) | 事件触发 | mqtt_bridge | summon_service |
| `/summon_result` | `std_msgs/String` | QoS(depth=1) | 事件触发 | summon_service | mqtt_bridge |
| `/mark_waypoint_label` | `std_msgs/String` | default | 事件触发 | mqtt_bridge | gps_path_recorder |
| `/path_graph/stats` | `std_msgs/String` | default | 5 | gps_path_recorder | mqtt_bridge |

**字段说明**：

- `/summon_request`：召唤请求，包含目标路点名称。
- `/summon_result`：召唤结果反馈，包含成功/失败状态及原因。
- `/mark_waypoint_label`：路点标记命令，为当前位置标注名称标签。
- `/path_graph/stats`：路点图统计信息，供 Web APP 实时显示。

---

## 3. 共享数据文件契约

除 ROS2 话题外，以下文件作为模块间的持久化数据接口：

| 文件路径 | 写入方 | 读取方 | 格式 |
|----------|--------|--------|------|
| `~/golf_ws/data/path_graph.json` | gps_path_recorder | summon_service, gps_waypoint_follower, generate_waypoint_map.py | JSON |

**`path_graph.json` 数据结构**：

```json
{
  "waypoints": {
    "wp_001": {
      "lat": 22.12345,
      "lon": 113.12345,
      "label": "发球台",
      "visits": 3
    }
  },
  "edges": [
    ["wp_001", "wp_002"],
    ["wp_002", "wp_003"]
  ],
  "counter": 42
}
```

**访问规则**：

- **写入方独占**：仅 gps_path_recorder 有写入权限，其他模块只读。
- **原子写入**：写入时先写临时文件再重命名，避免读取方读到不完整数据。
- **文件监控**：读取方应在需要时重新加载文件，不应缓存过期数据。

---

## 4. QoS 策略说明

### 4.1 QoS 类型定义

本系统使用以下 QoS 配置：

| 名称 | Reliability | Durability | History | Depth | 适用场景 |
|------|-------------|------------|---------|-------|----------|
| **default** | RELIABLE | VOLATILE | KEEP_LAST | 10 | 大多数话题的默认配置 |
| **BEST_EFFORT** | BEST_EFFORT | VOLATILE | KEEP_LAST | 10 | 传感器原始数据（允许丢帧以降低延迟） |
| **QoS(depth=1)** | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST | 1 | 事件型消息（确保后订阅者也能收到最新一条） |

### 4.2 QoS 兼容性规则

- 发布方使用 BEST_EFFORT 时，订阅方**必须**也使用 BEST_EFFORT，否则无法收到消息。
- 发布方使用 RELIABLE 时，订阅方可以使用 RELIABLE 或 BEST_EFFORT。
- 传感器驱动（GPS、LiDAR）使用 BEST_EFFORT 以保证实时性。
- 事件型命令话题（`/nav_trigger`、`/nav_complete`、`/summon_request`、`/summon_result`）使用 TRANSIENT_LOCAL 确保迟到的订阅方能收到最近的指令。

---

## 5. 变更流程

### 5.1 变更原则

接口契约的任何变更（包括但不限于话题名称、消息类型、QoS 策略、发布频率、新增/删除发布方或订阅方）均属于**破坏性变更**，必须遵循以下流程。

### 5.2 变更步骤

1. **提出变更**：变更发起人在项目看板创建变更请求，明确说明：
   - 变更的话题名称
   - 变更内容（旧值 → 新值）
   - 变更原因
   - 影响范围（受影响的所有模块及负责人）

2. **通知受影响方**：变更发起人必须逐一通知所有受影响模块的负责人，不得遗漏。

3. **达成一致**：所有受影响方确认变更方案后，方可进入实施阶段。任何一方有异议则需协商修改。

4. **同步实施**：所有受影响模块在同一开发周期内完成适配，避免接口不匹配导致系统故障。

5. **更新文档**：变更实施完成后，必须同步更新本文档，保持文档与代码一致。

6. **集成验证**：变更合入主分支前，必须通过全链路集成测试。

### 5.3 紧急变更

生产环境紧急修复允许先实施后补流程，但必须在 24 小时内完成文档更新和受影响方通知。

---

## 6. 附录：模块-话题依赖矩阵

下表列出每个模块发布（P）和订阅（S）的话题关系，便于快速定位变更影响范围。

| 模块 | 角色 | 关联话题 |
|------|------|----------|
| **yolo_ros** | 目标检测 | P: `/yolo/tracking` |
| **detection_node** | 检测输出 | P: `/detections`, `/detection_keypoints` |
| **gesture_node** | 手势识别 | S: `/yolo/tracking` → P: `/gesture_cmd` |
| **lock_manager** | 目标锁定 | S: `/yolo/tracking` → P: `/locked_target`, `/follow_state` |
| **mode_manager** | 模式管理 | S: `/gesture_cmd`, `/follow_state`, `/nav_trigger`, `/joy` → P: `/system_mode` |
| **follow_target_publisher** | 跟随控制 | S: `/locked_target`, `/follow_state`, `/system_mode`, `/lx_camera_node/LxCamera_Depth` → P: `/cmd_vel` |
| **gps_waypoint_follower** | GPS 导航 | S: `/system_mode`, `/gps/fix`, `/heading_deg`, `/scan`, `/nav_trigger`, `/nav_blocked` → P: `/cmd_vel_nav`, `/nav_complete` |
| **lidar_emergency_stop** | 急停中继 | S: `/cmd_vel_nav`, `/scan` → P: `/cmd_vel`, `/nav_blocked`, `/emergency_stop_state` |
| **gps_path_recorder** | 路点记录 | S: `/gps/fix`, `/gps/filtered`, `/follow_state`, `/odom_combined`, `/mark_waypoint_label` → P: `/path_graph/stats`；写入: `path_graph.json` |
| **mqtt_bridge** | MQTT 桥接 | S: `/system_mode`, `/follow_state`, `/gps/fix`, `/nav_complete`, `/path_graph/stats`, `/summon_result` → P: `/nav_trigger`, `/summon_request`, `/mark_waypoint_label` |
| **summon_service** | 召唤服务 | S: `/summon_request`, `/nav_complete` → P: `/nav_trigger`, `/summon_result`；读取: `path_graph.json` |
| **imu_ned_to_enu** | IMU 坐标转换 | S: `/heading`, `/h30_imu_raw` → P: `/h30_imu` |
| **GPS 驱动** | GPS 数据源 | P: `/gps/fix`, `/heading`, `/heading_deg` |
| **S11 驱动** | 相机数据源 | P: `/lx_camera_node/LxCamera_Rgb`, `/lx_camera_node/LxCamera_Depth` |
| **N10P 驱动** | LiDAR 数据源 | P: `/scan` |
| **yesense 驱动** | IMU 数据源 | P: `/h30_imu_raw` |
| **手柄驱动** | 手柄输入 | P: `/joy` |
| **底盘驱动** | 底盘执行 | S: `/cmd_vel` |

---

*本文档由项目团队维护，任何接口变更须同步更新此文档。*
