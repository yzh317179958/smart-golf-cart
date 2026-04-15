# 智能高尔夫球车自动驾驶系统 — 软件架构文档

> **版本**: v8.2.7 | **平台**: ROS2 Humble | **最后更新**: 2026-04-07

---

## 1. 系统概览

本系统为高尔夫球车提供自动驾驶能力，核心功能包括：

- **人员跟随**: 基于视觉检测自动跟随目标人员，PID 控制行驶
- **GPS 路点导航**: 基于预录路径图的 Dijkstra 最短路径规划 + PID 航向控制
- **远程召唤**: 手机 APP 通过 MQTT 4G 通信发起导航/急停/模式切换
- **路径建图**: 跟随过程中自动记录 GPS 路点，构建路径拓扑图

系统采用分层架构，感知、导航、建图、通信各层独立，通过 ROS2 话题松耦合。
全局状态机 `mode_manager` 统一协调各子系统的工作模式。

底盘为阿克曼转向（Ackermann），非差速，影响转向控制策略。

---

## 2. 硬件拓扑

```
                          ┌──────────────────────────────────┐
                          │   Jetson Orin NX 16GB            │
                          │   JetPack 6.1 / Ubuntu 22.04    │
                          │   ROS2 Humble                    │
                          ├──────────────────────────────────┤
                          │                                  │
    ┌─────────────┐  USB  │  /dev/wheeltec_controller        │
    │ 底盘 STM32  │◄─────►│  (串口, CH343)                   │
    │ R550 AKM+   │       │  阿克曼转向 + 轮速里程计          │
    └─────────────┘       │                                  │
                          │                                  │
    ┌─────────────┐  USB  │  /dev/wheeltec_lidar             │
    │ N10P LiDAR  │──────►│  (串口, CH343)                   │
    │ 2D 360°     │       │  /scan (LaserScan)               │
    └─────────────┘       │                                  │
                          │                                  │
    ┌─────────────┐  USB  │  /dev/wheeltec_gnss              │
    │ G90 GPS     │──────►│  (串口, CH343, 115200 baud)      │
    │ UM982 双天线 │       │  /gps/fix + /heading             │
    │ ANT1前+ANT2后│       │  水平 ±1.5m CEP, 航向 ±0.3°     │
    └─────────────┘       │                                  │
                          │                                  │
    ┌─────────────┐  USB  │  /h30_imu_raw (Imu)             │
    │ H30 AHRS    │──────►│  9轴惯性测量单元                  │
    │ 9-axis IMU  │       │  200Hz, NED坐标系                │
    └─────────────┘       │                                  │
                          │                                  │
    ┌─────────────┐  以太网│  /lx_camera_node/LxCamera_Rgb   │
    │ MRDVS S11   │──────►│  RGB 1280x1080                   │
    │ RGB + ToF    │       │  /lx_camera_node/LxCamera_Depth │
    │              │       │  ToF 深度 240x90 (16UC1)        │
    └─────────────┘       │                                  │
                          │                                  │
    ┌─────────────┐  USB  │  usb0 RNDIS                     │
    │ 4G Dongle   │──────►│  反向 SSH 隧道 → VPS             │
    │ EC20F       │       │  MQTT 4G 通信                    │
    └─────────────┘       │                                  │
                          └──────────────────────────────────┘
                                        │
                                   4G 网络
                                        │
                                        ▼
                              ┌──────────────────┐
                              │  云端 VPS         │
                              │  MQTT Broker     │
                              │  SSH 管理         │
                              └──────────────────┘
                                        │
                                   4G 网络
                                        │
                                        ▼
                              ┌──────────────────┐
                              │  手机 APP         │
                              │  (Leaflet 地图)   │
                              │  召唤/导航/监控    │
                              └──────────────────┘
```

---

## 3. 软件包结构

```
golf_ws/src/
├── golf_perception/              # 感知层
│   ├── detection_node.py         #   YOLOv8-Pose 行人检测
│   ├── gesture_node.py           #   手势识别（举手 → lock/stop）
│   ├── camera_info_fix.py        #   S11 深度 CameraInfo 修复
│   └── image_resizer.py          #   调试图像缩放（→ /dbg_small）
│
├── golf_navigation/              # 导航层
│   ├── mode_manager_node.py      #   全局状态机（FOLLOWING/NAVIGATION/E_STOP）
│   ├── follow_target_publisher.py#   跟随 PID 控制器
│   ├── lock_manager_node.py      #   目标锁定管理
│   ├── gps_waypoint_follower.py  #   GPS 脉冲转向路点跟随（内置 LiDAR 避墙）
│   └── summon_service.py         #   远程召唤服务（APP → 导航）
│
├── golf_mapping/                 # 建图层
│   ├── gps_path_recorder.py      #   GPS 路点自动记录 + 路径图维护
│   ├── imu_ned_to_enu.py         #   IMU NED→ENU 坐标转换 + G90 航向融合
│   └── test_logger.py            #   实车测试日志记录器
│
├── golf_communication/           # 通信层
│   └── mqtt_bridge_node.py       #   MQTT ↔ ROS2 双向桥接
│
├── golf_bringup/                 # 启动层
│   ├── launch/                   #   Launch 文件集
│   │   ├── sensors.launch.py     #     传感器层（底盘、LiDAR、GPS、IMU、S11）
│   │   ├── perception.launch.py  #     感知层（YOLO、手势、锁定）
│   │   ├── follow.launch.py      #     跟随 + GPS 导航 + 召唤 + MQTT（含 mode 参数）
│   │   ├── slam.launch.py        #     SLAM 建图
│   │   └── ...                   #     其他辅助 launch
│   ├── config/                   #     参数配置文件
│   └── urdf/                     #     机器人模型描述
│
└── third_party/
    └── lx_camera_ros/            # MRDVS S11 相机 ROS2 驱动（第三方）
```

**环境继承链**:

```
/opt/ros/humble → ~/wheeltec_ros2/install → ~/golf_ws/install
```

WheelTec 官方包提供底盘驱动、GPS 驱动、EKF、Nav2 等基础能力，本项目仅编写上层应用节点。

---

## 4. 数据流图

### 4.1 全局数据流总览

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              感知层 (Perception)                            │
│                                                                             │
│  S11 RGB ───► [detection_node] ──► /detections (Detection2DArray)           │
│  1280x1080       YOLOv8-Pose       /detection_keypoints (String/JSON)      │
│                                                                             │
│  /yolo/tracking ◄── [yolo_ros*]  ◄── /detections                           │
│  (DetectionArray)    外部YOLO跟踪                                            │
│                                                                             │
│  /yolo/tracking ──► [gesture_node] ──► /gesture_cmd (String)               │
│                      手势: lock/stop/none                                   │
│                                                                             │
│  /yolo/tracking ──► [lock_manager] ──► /locked_target (Detection)          │
│  /gesture_cmd   ──►   目标锁定         /follow_state (String)              │
│                                         idle/tracking/paused                │
└─────────────────────────────────────────────────────────────────────────────┘
         │ /locked_target                        │ /follow_state
         │ /follow_state                         │ /gesture_cmd
         ▼                                       ▼
┌──────────────────────────────┐  ┌──────────────────────────────────────────┐
│        跟随层 (Follow)        │  │            状态管理 (State)               │
│                              │  │                                          │
│  /locked_target ─►           │  │  /follow_state ──►                       │
│  S11 Depth     ─►            │  │  /gesture_cmd  ──►                       │
│  /system_mode  ─►            │  │  /nav_trigger  ──► [mode_manager]        │
│  [follow_target_publisher]   │  │  /nav_complete ──►     状态机             │
│    PID 控制                   │  │  /joy          ──►                       │
│         │                    │  │                       │                   │
│         ▼                    │  │                       ▼                   │
│  /cmd_vel (Twist)            │  │  /system_mode (String) 10Hz              │
│  仅 FOLLOWING 模式时输出      │  │  following / navigation / e_stop         │
└──────────────────────────────┘  └──────────────────────────────────────────┘
                                              │ /system_mode
                                              ▼ (所有节点读取)
┌─────────────────────────────────────────────────────────────────────────────┐
│                              导航层 (Navigation)                            │
│                                                                             │
│  /nav_trigger (String) ──► [gps_waypoint_follower]                         │
│  /gps/fix (NavSatFix)  ──►   Dijkstra + v8.3 脉冲转向                      │
│  /heading_deg (Float32) ──►   前瞻 +1 方位角 + 死区脉冲                     │
│  /scan (LaserScan)      ──►   前方 ±75° 侧边护栏 → 避墙脉冲                 │
│                                    │                                        │
│                                    ▼                                        │
│                            /cmd_vel (Twist)                                 │
│                       （launch 里 cmd_vel_topic:=/cmd_vel，直发底盘）        │
│                                                                             │
│  导航完成 ──► /nav_complete (String)                                        │
│  阻断信号 ◄── /nav_blocked (String)  （外部预留接口）                        │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│                              建图层 (Mapping)                               │
│                                                                             │
│  G90 /heading (QuaternionStamped)  ──►                                     │
│  H30 /h30_imu_raw (Imu)           ──► [imu_ned_to_enu]                    │
│                                          G90优先, H30回退                   │
│                                              │                              │
│                                              ▼                              │
│                                        /h30_imu (Imu, ENU)                 │
│                                                                             │
│  /gps/fix (NavSatFix)  ──►                                                 │
│  /follow_state (String) ──► [gps_path_recorder]                            │
│  /odom_combined (Odometry) ──►  路点记录 + 路径图维护                        │
│  TF odom_combined→base_footprint ──►                                       │
│                                        │                                    │
│                                        ▼                                    │
│                               path_graph.json (磁盘文件)                    │
│                               /path_graph/stats (String/JSON)               │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│                              通信层 (Communication)                         │
│                                                                             │
│  车 → APP (ROS2 订阅 → MQTT 发布):                                         │
│    /system_mode   ──►                                                       │
│    /gps/fix       ──►                                                       │
│    /follow_state  ──► [mqtt_bridge] ──► MQTT Broker ──► 手机 APP           │
│    /nav_complete  ──►                                                       │
│    /summon_result ──►                                                       │
│    /path_graph/stats ──►                                                    │
│                                                                             │
│  APP → 车 (MQTT 订阅 → ROS2 发布):                                         │
│    手机 APP ──► MQTT Broker ──► [mqtt_bridge]                              │
│                                    ──► /summon_request (String/JSON)        │
│                                    ──► /mark_waypoint_label (String)        │
│                                    ──► /nav_trigger (String)                │
│                                                                             │
│  召唤处理:                                                                   │
│    /summon_request ──► [summon_service] ──► /nav_trigger                    │
│    /nav_complete   ──►                 ──► /summon_result (String/JSON)     │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 4.2 /cmd_vel 输出路径（互斥）

跟随模式和导航模式的速度指令走不同路径，最终汇聚到 `/cmd_vel`:

```
FOLLOWING 模式:
  [follow_target_publisher] ──► /cmd_vel (直接)

NAVIGATION 模式:
  [gps_waypoint_follower] ──► /cmd_vel  (脉冲直发，内置 LiDAR 避墙)

E_STOP 模式:
  [mode_manager] ──► /cmd_vel = Twist() 零速度
```

### 4.3 话题清单

| 话题名 | 消息类型 | 发布者 | 订阅者 | 频率 |
|--------|---------|--------|--------|------|
| `/system_mode` | `std_msgs/String` | mode_manager | 全部节点 | 2 Hz |
| `/follow_state` | `std_msgs/String` | lock_manager | mode_manager, follow_target_publisher, gps_path_recorder, mqtt_bridge | 10 Hz |
| `/locked_target` | `yolo_msgs/Detection` | lock_manager | follow_target_publisher | ~15 Hz |
| `/gesture_cmd` | `std_msgs/String` | gesture_node | lock_manager, mode_manager | ~15 Hz |
| `/cmd_vel` | `geometry_msgs/Twist` | follow_target_publisher / gps_waypoint_follower / mode_manager | 底盘 STM32 | 10-20 Hz |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | gps_waypoint_follower (默认话题，launch 重映射为 /cmd_vel) | — | 10 Hz |
| `/nav_trigger` | `std_msgs/String` | summon_service / mqtt_bridge | mode_manager, gps_waypoint_follower | 事件 |
| `/nav_complete` | `std_msgs/String` | gps_waypoint_follower | mode_manager, summon_service, mqtt_bridge | 事件 |
| `/nav_blocked` | `std_msgs/String` | 外部预留 | gps_waypoint_follower | 事件 |
| `/summon_request` | `std_msgs/String` | mqtt_bridge | summon_service | 事件 |
| `/summon_result` | `std_msgs/String` | summon_service | mqtt_bridge | 事件 |
| `/gps/fix` | `sensor_msgs/NavSatFix` | GPS 驱动 | gps_waypoint_follower, gps_path_recorder, mqtt_bridge | 1-5 Hz |
| `/heading` | `geometry_msgs/QuaternionStamped` | GPS 驱动 | imu_ned_to_enu | 1-5 Hz |
| `/heading_deg` | `std_msgs/Float32` | GPS 驱动 | gps_waypoint_follower, test_logger | 1-5 Hz |
| `/h30_imu_raw` | `sensor_msgs/Imu` | H30 驱动 | imu_ned_to_enu | 200 Hz |
| `/h30_imu` | `sensor_msgs/Imu` | imu_ned_to_enu | EKF (robot_localization) | 200 Hz |
| `/scan` | `sensor_msgs/LaserScan` | N10P 驱动 | gps_waypoint_follower | 10 Hz |
| `/lx_camera_node/LxCamera_Rgb` | `sensor_msgs/Image` | S11 驱动 | detection_node | ~15 Hz |
| `/lx_camera_node/LxCamera_Depth` | `sensor_msgs/Image` | S11 驱动 | follow_target_publisher | ~15 Hz |
| `/detections` | `vision_msgs/Detection2DArray` | detection_node | yolo_ros | ~15 Hz |
| `/yolo/tracking` | `yolo_msgs/DetectionArray` | yolo_ros | gesture_node, lock_manager | ~15 Hz |
| `/odom_combined` | `nav_msgs/Odometry` | EKF | gps_path_recorder | 50 Hz |
| `/joy` | `sensor_msgs/Joy` | 手柄驱动 | mode_manager | 50 Hz |
| `/path_graph/stats` | `std_msgs/String` | gps_path_recorder | mqtt_bridge | 0.2 Hz |
| `/mark_waypoint_label` | `std_msgs/String` | mqtt_bridge | gps_path_recorder | 事件 |

---

## 5. 状态机说明

系统由 `mode_manager` 节点维护全局状态，通过 `/system_mode` 话题以 2Hz 广播。
所有下游节点根据当前模式决定是否工作。

### 5.1 状态定义

| 状态 | 含义 | 活跃子系统 |
|------|------|-----------|
| `following` | 人员跟随模式（默认） | 感知 + 锁定 + 跟随控制 |
| `navigation` | GPS 路点导航模式 | GPS导航 + LiDAR急停 |
| `e_stop` | 紧急停车 | 全部停止，零速度输出 |

### 5.2 状态转换图

```
                    ┌─────────────────────────────────────┐
                    │          手柄急停按钮                 │
                    │          目标丢失 >15s               │
                    │          APP 远程 e_stop             │
                    ▼                                     │
              ┌──────────┐                                │
              │          │                                │
    ┌────────►│  E_STOP  │◄───────────────────────┐       │
    │         │          │                        │       │
    │         └────┬─────┘                        │       │
    │              │                              │       │
    │  手柄恢复按钮 │                              │       │
    │  APP resume  │                              │       │
    │              ▼                              │       │
    │    ┌────────────────┐   /nav_trigger   ┌────┴──────────┐
    │    │                │ ───────────────► │               │
    │    │   FOLLOWING    │                  │  NAVIGATION   │
    │    │   (默认模式)    │ ◄─────────────── │               │
    │    │                │  /nav_complete   │               │
    │    └────────────────┘  APP cancel      └───────────────┘
    │              │         lock 手势                │
    │              │                                  │
    │              └──────────────────────────────────┘
    │                     (见上方触发条件)
    └─────────────── 手柄急停 / 丢失超时 / APP ──────────┘
```

### 5.3 转换触发条件

| 转换 | 触发源 | 话题/按钮 |
|------|--------|----------|
| FOLLOWING → NAVIGATION | APP 召唤 / 手动触发 | `/nav_trigger` (目标名或路点ID) |
| NAVIGATION → FOLLOWING | 导航到达 | `/nav_complete` |
| NAVIGATION → FOLLOWING | APP 取消 | `/nav_trigger` = "cancel" |
| NAVIGATION → FOLLOWING | Lock 手势 | `/gesture_cmd` = "lock" |
| 任意 → E_STOP | 手柄急停按钮 | `/joy` buttons[0] |
| 任意 → E_STOP | 目标丢失超时 | `/follow_state` lost >15s |
| 任意 → E_STOP | APP 远程急停 | `/nav_trigger` = "e_stop" |
| E_STOP → FOLLOWING | 手柄恢复按钮 | `/joy` buttons[1] |
| E_STOP → FOLLOWING | APP 恢复 | `/nav_trigger` = "resume" |

---

## 6. 节点说明表

### 6.1 感知层 (golf_perception)

| 节点名 | 文件 | 功能 |
|--------|------|------|
| `detection_node` | detection_node.py | 订阅 S11 RGB，YOLOv8s-Pose TensorRT 推理，发布行人检测 bbox 和 17 关键点 |
| `gesture_node` | gesture_node.py | 基于 YOLO Pose 关键点判定举手手势（右手→lock，左手→stop），含帧计数防误判 |
| `camera_info_fix` | camera_info_fix.py | 修复 S11 深度话题的非标准 CameraInfo 和 encoding，适配 depthimage_to_laserscan |
| `image_resizer` | image_resizer.py | 将 YOLO 调试图像缩放为 320x270 发布到 /dbg_small，降低远程可视化带宽 |

### 6.2 导航层 (golf_navigation)

| 节点名 | 文件 | 功能 |
|--------|------|------|
| `mode_manager` | mode_manager_node.py | 全局三态状态机，综合手柄/手势/导航/APP 输入，广播 /system_mode 供所有节点读取 |
| `follow_target_publisher` | follow_target_publisher.py | 跟随控制器，从 /locked_target bbox + S11 深度计算角度和距离，PID 输出 /cmd_vel |
| `lock_manager` | lock_manager_node.py | 自动锁定最大人体 bbox，维护 tracking/idle/paused 状态，发布 /locked_target 和 /follow_state |
| `gps_waypoint_follower` | gps_waypoint_follower.py | GPS 脉冲转向路点跟随，Dijkstra 最短路径规划 + 前瞻 +1 死区脉冲 + LiDAR ±75° 避墙脉冲，直发 /cmd_vel |
| `summon_service` | summon_service.py | 接收 APP 召唤请求（球洞名或 GPS 坐标），解析后发送 /nav_trigger 启动导航 |

### 6.3 建图层 (golf_mapping)

| 节点名 | 文件 | 功能 |
|--------|------|------|
| `gps_path_recorder` | gps_path_recorder.py | 跟随行驶中自动记录 GPS 路点，构建路径拓扑图 (path_graph.json)，含跨会话帧对齐和加权平均 |
| `imu_ned_to_enu` | imu_ned_to_enu.py | 融合 G90 双天线航向 (±0.3°) 和 H30 磁力计，输出 ENU 坐标系 IMU 数据供 EKF 使用 |
| `test_logger` | test_logger.py | 实车测试时后台记录 GPS 轨迹 + 系统事件到 JSON 文件，用于离线分析和可视化 |

### 6.4 通信层 (golf_communication)

| 节点名 | 文件 | 功能 |
|--------|------|------|
| `mqtt_bridge` | mqtt_bridge_node.py | ROS2 ↔ MQTT 双向桥接，6 路车→APP 状态推送 + 3 路 APP→车 命令接收，线程安全队列设计 |

### 6.5 外部依赖节点（WheelTec 官方 / 第三方）

| 节点 | 来源 | 功能 |
|------|------|------|
| `wheeltec_robot_node` | wheeltec_ros2 | 底盘驱动，订阅 /cmd_vel，发布 /odom、TF |
| `wheeltec_gps_driver` | wheeltec_ros2 | G90 GPS NMEA 驱动，发布 /gps/fix、/heading、/heading_deg |
| `yesense_imu_node` | wheeltec_ros2 | H30 IMU 驱动，发布 /h30_imu_raw |
| `lx_camera_node` | third_party/lx_camera_ros | S11 相机驱动，发布 RGB 和 ToF 深度图 |
| `n10p_lidar_node` | wheeltec_ros2 | N10P LiDAR 驱动，发布 /scan |
| `ekf_localization_node` | robot_localization | 扩展卡尔曼滤波，融合轮速+IMU，输出 /odom_combined 和 TF |
| `yolo_ros` | yolo_ros (外部) | YOLO 跟踪封装，输入 /detections，输出 /yolo/tracking（含 track ID） |
| `joy_node` | ros2 joy | 手柄驱动，发布 /joy |

---

## 附录 A: 关键设计决策

| 决策 | 选择 | 理由 |
|------|------|------|
| GPS 导航方案 | GPS-PID 直控，非 Nav2 | GPS ±1.5m 噪声下 PID 天然低通滤波，MPPI 对 GPS 路径偏移敏感 |
| 跟随控制 | 视觉 PID 直发 /cmd_vel | 简单可靠，bbox 中心 + 深度 → 角度/距离 → PID，复用 WheelTec 官方 PID |
| 航向源 | G90 双天线优先，H30 回退 | G90 ±0.3° 全天候可用，不受磁场干扰；H30 磁力计仅作降级备份 |
| 路点坐标 | 裸 GPS + 多次经过加权平均 | 避免 EKF 长期漂移污染路点，多次采样统计平均提升精度 |
| 远程通信 | MQTT over 4G | 轻量级、低延迟、成熟生态、手机 APP 原生支持 |
| 安全兜底 | LiDAR 急停门控 + 手柄 E_STOP | 多层安全：软件急停（LiDAR 前方检测）+ 硬件急停（手柄物理按钮） |

## 附录 B: 文件存储

| 文件 | 路径 | 内容 |
|------|------|------|
| 路径拓扑图 | `~/golf_ws/data/path_graph.json` | 路点坐标、边、经过次数、标签 |
| 测试日志 | `~/golf_ws/data/test_log_*.json` | GPS 轨迹 + 系统事件时序 |
| YOLOv8 模型 | 配置指定路径 | TensorRT 引擎文件 (.engine) |
