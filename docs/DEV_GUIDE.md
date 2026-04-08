# Smart Golf Cart 开发者指南

> **版本**: v1.0 | **适用平台**: ROS2 Humble / Ubuntu 22.04 | **最后更新**: 2026-04-07

---

## 目录

1. [团队分工](#1-团队分工)
2. [环境搭建](#2-环境搭建)
3. [Git 工作流](#3-git-工作流)
4. [代码规范](#4-代码规范)
5. [话题接口约定](#5-话题接口约定)
6. [测试流程](#6-测试流程)
7. [项目负责人集成流程](#7-项目负责人集成流程)
8. [常见问题](#8-常见问题)

---

## 1. 团队分工

### 1.1 包归属表

| Package | 负责人 | 职责描述 |
|---------|--------|----------|
| `golf_perception` | 图像识别工程师 | YOLO 目标检测、手势识别、深度图修复、图像缩放 |
| `golf_navigation` | 规控工程师 | 跟随控制、GPS-PID 导航、模式管理、LiDAR 急停、召唤服务 |
| `golf_mapping` | 地图+传感器工程师 | GPS 路点记录、IMU NED/ENU 融合、测试日志 |
| `golf_communication` | 项目负责人 | MQTT 桥接（Web APP 通信） |
| `golf_bringup` | 项目负责人 | Launch 文件、配置文件、URDF、启停脚本 |

### 1.2 各角色职责

**项目负责人**
- 管理 `golf_bringup` 和 `golf_communication`
- SLAM 建图与实车测试
- PR 审核与 main 分支合并
- 每 1-2 周组织实车联调

**规控工程师**
- 维护 `golf_navigation` 内所有节点
- 包括: `mode_manager`、`follow_target_publisher`、`gps_waypoint_follower`、`lidar_emergency_stop`、`lock_manager`、`summon_service`

**图像识别工程师**
- 维护 `golf_perception` 内所有节点
- 包括: `detection_node`、`gesture_node`、`camera_info_fix`、`image_resizer`

**地图+传感器工程师**
- 维护 `golf_mapping` 内所有节点
- 包括: `gps_path_recorder`、`imu_ned_to_enu`、`test_logger`

---

## 2. 环境搭建

### 2.1 基础环境（所有人）

**操作系统**: Ubuntu 22.04 LTS

**安装 ROS2 Humble**:
```bash
# 按照官方文档安装: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions python3-rosdep

# 必备 ROS2 包
sudo apt install ros-humble-cv-bridge ros-humble-image-transport \
    ros-humble-vision-msgs ros-humble-nav2-msgs ros-humble-sensor-msgs \
    ros-humble-geometry-msgs ros-humble-std-msgs
```

**Python 依赖**:
```bash
pip3 install paho-mqtt transforms3d numpy opencv-python
```

**克隆仓库并编译**:
```bash
mkdir -p ~/golf_ws/src && cd ~/golf_ws/src
git clone <仓库地址> .

# 编译（symlink-install 方便开发调试）
cd ~/golf_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 加入环境（建议写入 ~/.bashrc）
source ~/golf_ws/install/setup.bash
```

### 2.2 图像识别工程师额外依赖

图像识别工程师需要在 Jetson 上运行 YOLO 推理，本地开发机也需安装以便调试。

**yolo_ros 包**（第三方 YOLO ROS2 封装）:
```bash
cd ~/golf_ws/src
git clone https://github.com/mgonzs13/yolo_ros.git
```

**Jetson 推理依赖**:
```bash
# ultralytics (YOLO 框架)
pip3 install ultralytics

# PyTorch — Jetson 专用 wheel，参考 NVIDIA 官方
# https://forums.developer.nvidia.com/t/pytorch-for-jetson/
pip3 install torch-<version>-cp310-cp310-linux_aarch64.whl

# TensorRT — JetPack 6.1 自带，确认版本
dpkg -l | grep tensorrt
python3 -c "import tensorrt; print(tensorrt.__version__)"
```

**本地开发机（x86 模拟调试）**:
```bash
pip3 install ultralytics torch torchvision
# 注: 本地无 TensorRT 加速，仅用于逻辑验证
```

### 2.3 编译单个包（日常开发）

```bash
# 只编译自己的包，速度快
colcon build --symlink-install --packages-select golf_perception
colcon build --symlink-install --packages-select golf_navigation
colcon build --symlink-install --packages-select golf_mapping
```

---

## 3. Git 工作流

### 3.1 分支策略

- **`main` 分支受保护**：只有项目负责人可以合并
- 所有开发在 `feature/xxx` 分支上进行
- 禁止直接向 main push

### 3.2 开发流程

```bash
# 1. 从最新 main 创建功能分支
git checkout main
git pull origin main
git checkout -b feature/your-feature

# 2. 开发 + 本地验证（见第 6 节测试流程）

# 3. 提交（遵循 commit message 规范）
git add <你修改的文件>
git commit -m "feat(navigation): add pure pursuit fallback mode"

# 4. 推送到远程
git push origin feature/your-feature

# 5. 在 GitHub 创建 Pull Request
#    - 标题简洁，说明改了什么
#    - 描述中附上验证截图或日志
#    - 指定项目负责人为 Reviewer

# 6. 项目负责人 review 通过后 merge 到 main
```

### 3.3 Commit Message 规范

格式: `type(scope): description`

**type 类型**:

| type | 用途 | 示例 |
|------|------|------|
| `feat` | 新功能 | `feat(perception): add hand gesture stop command` |
| `fix` | 修复 bug | `fix(navigation): fix GPS heading wraparound at 360 degrees` |
| `tune` | 参数调优 | `tune(navigation): reduce arrival_tolerance to 1m` |
| `refactor` | 重构（不改功能） | `refactor(mapping): extract GPS filter to separate method` |
| `docs` | 文档 | `docs(bringup): update launch file comments` |

**scope 范围**（包名，不带 golf_ 前缀）:
- `perception` / `navigation` / `mapping` / `communication` / `bringup`

**示例**:
```
feat(navigation): add pure pursuit fallback mode
fix(perception): handle empty detection array gracefully
tune(mapping): increase min_record_distance to 3m
refactor(communication): simplify MQTT reconnect logic
docs(bringup): document sensor launch parameters
```

### 3.4 PR 规范

> PR 创建时会自动填充模板（`.github/PULL_REQUEST_TEMPLATE.md`），**所有字段必填**，缺项将被打回。

**标题**: 与主要 commit message 一致即可

**描述必填三要素**:

| 要素 | 说明 | 示例 |
|------|------|------|
| **改了什么** | 具体修改的文件和内容 | `follow_target_publisher.py: PID angular_kd 0.005→0.01` |
| **为什么改** | 改动的动机和目标（关联 Issue 编号） | `转弯响应慢 0.5s，增大微分项提升响应 #12` |
| **怎么验证的** | 验证方法和结果数据 | `rosbag 回放：转弯响应 0.5s→0.2s，附截图` |

**PR 不合格将被打回的情况**:
- 缺少"为什么改"（只写了改什么，不写动机）
- 缺少验证截图或数据对比
- 修改了不属于自己的包的文件
- 改了话题接口但没有提前通知

---

## 4. 代码规范

### 4.1 Python 风格

- 遵循 **PEP 8** 规范
- 缩进使用 4 空格（禁止 Tab）
- 行宽上限 120 字符
- 类名 `CamelCase`，函数/变量 `snake_case`
- 每个文件顶部加简要注释说明用途

### 4.2 ROS2 命名规范

| 类型 | 命名方式 | 示例 |
|------|----------|------|
| 节点名 | snake_case | `mode_manager`、`detection_node` |
| 话题名 | /snake_case | `/follow_state`、`/cmd_vel` |
| 服务名 | /snake_case | `/summon_request` |
| 参数名 | snake_case | `arrival_tolerance`、`max_speed` |
| 消息类型 | CamelCase | `Twist`、`NavSatFix`、`DetectionArray` |
| 文件名 | snake_case.py | `mode_manager_node.py` |
| 包名 | snake_case | `golf_navigation` |

### 4.3 节点注册

每个新节点**必须**在对应包的 `setup.py` 中注册 `entry_points`。例如:

```python
# golf_navigation/setup.py
entry_points={
    'console_scripts': [
        'mode_manager = golf_navigation.mode_manager_node:main',
        'follow_target_publisher = golf_navigation.follow_target_publisher:main',
        # 新节点在这里加一行
        'your_new_node = golf_navigation.your_new_node:main',
    ],
},
```

注册后需重新编译:
```bash
colcon build --symlink-install --packages-select golf_navigation
```

### 4.4 铁律

1. **不要修改其他人负责的包内文件** -- 需要改请先沟通，由对应负责人修改或批准
2. **参数必须通过 launch 文件传入**，禁止在代码中硬编码
3. **禁止硬编码 IP 地址、密码、文件路径** -- 全部参数化
4. **话题名和消息类型变更必须提前通知所有人** -- 这是包间接口契约
5. **增量开发** -- 每次只改一个功能点，验证通过再改下一个
6. **不要自己造轮子** -- 优先复用 WheelTec 官方代码和 ROS2 生态成熟方案

---

## 5. 话题接口约定

以下是各包之间的关键话题接口。**修改任何话题名或消息类型前必须通知全团队**。

### 5.1 感知层输出（golf_perception）

| 话题 | 消息类型 | 方向 | 说明 |
|------|----------|------|------|
| `/yolo/tracking` | `yolo_msgs/DetectionArray` | 输出 | YOLO 跟踪结果（由 yolo_ros 发布） |
| `/gesture_cmd` | `std_msgs/String` | 输出 | 手势指令（"stop" 等） |
| `/camera_info_fixed` | `sensor_msgs/CameraInfo` | 输出 | 修正后的相机内参 |
| `/dbg_small` | `sensor_msgs/Image` | 输出 | 调试用缩小图 |

### 5.2 导航控制层（golf_navigation）

| 话题 | 消息类型 | 方向 | 说明 |
|------|----------|------|------|
| `/system_mode` | `std_msgs/String` | 输出 | 当前系统模式（idle/follow/navigate） |
| `/follow_state` | `std_msgs/String` | 输出 | 跟随状态（tracking/lost/stopped） |
| `/locked_target` | `yolo_msgs/Detection` | 输出 | 锁定的跟随目标 |
| `/cmd_vel` | `geometry_msgs/Twist` | 输出 | 最终速度指令 |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | 中间 | 导航层速度（经急停中继） |
| `/nav_trigger` | `std_msgs/String` | 输入/输出 | 导航触发指令 |
| `/nav_complete` | `std_msgs/String` | 输出 | 导航完成通知 |
| `/nav_blocked` | `std_msgs/String` | 输出 | LiDAR 检测到前方障碍 |
| `/emergency_stop_state` | `std_msgs/String` | 输出 | 急停状态 |

### 5.3 建图传感器层（golf_mapping）

| 话题 | 消息类型 | 方向 | 说明 |
|------|----------|------|------|
| `/gps/fix` | `sensor_msgs/NavSatFix` | 输入 | 裸 GPS 定位 |
| `/heading_deg` | `std_msgs/Float32` | 输入 | G90 双天线航向角 |
| `/h30_imu` | `sensor_msgs/Imu` | 输出 | NED→ENU 转换后的 IMU 数据 |
| `/path_graph/stats` | `std_msgs/String` | 输出 | 路径图统计信息 |
| `/mark_waypoint_label` | `std_msgs/String` | 输入 | 标记路点（来自 MQTT） |

### 5.4 通信层（golf_communication）

| 话题 | 消息类型 | 方向 | 说明 |
|------|----------|------|------|
| `/summon_request` | `std_msgs/String` | 输出 | 召唤请求（MQTT → ROS2） |
| `/mark_waypoint_label` | `std_msgs/String` | 输出 | 标记路点（MQTT → ROS2） |
| `/nav_trigger` | `std_msgs/String` | 输出 | 导航触发（MQTT → ROS2） |

### 5.5 数据流概览

```
[GPS G90] → /gps/fix, /heading_deg
[H30 IMU] → /h30_imu_raw → imu_ned_to_enu → /h30_imu
[S11 相机] → /lx_camera_node/LxCamera_RGB, LxCamera_Depth
[N10P LiDAR] → /scan

/lx_camera_node/LxCamera_RGB → yolo_ros → /yolo/tracking
                                         → /yolo/dbg_image → image_resizer → /dbg_small

/yolo/tracking → lock_manager → /locked_target → follow_target_publisher → /cmd_vel
               → gesture_node → /gesture_cmd → mode_manager → /system_mode

/gps/fix + /heading_deg → gps_waypoint_follower → /cmd_vel_nav → lidar_emergency_stop → /cmd_vel

Web APP ← MQTT → mqtt_bridge → /nav_trigger, /summon_request, /mark_waypoint_label
```

---

## 6. 测试流程

### 6.1 本地编译验证（每次提交前必做）

```bash
# 语法检查
python3 -m py_compile golf_perception/golf_perception/your_file.py

# 编译自己的包
cd ~/golf_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select your_package

# 确认编译无 warning
```

### 6.2 Rosbag 回放测试

项目负责人会提供标准测试 bag 文件，放在 `data/` 目录下。

```bash
# 终端 1: 回放 bag（带时钟模拟）
ros2 bag play data/follow_bag_xxx --clock

# 终端 2: 启动你的节点（使用仿真时间）
source ~/golf_ws/install/setup.bash
ros2 run your_package your_node --ros-args -p use_sim_time:=true

# 终端 3: 验证输出
ros2 topic echo /your_output_topic
ros2 topic hz /your_output_topic
```

### 6.3 单节点隔离测试

如果你的节点不依赖硬件，可以手动发布测试消息:

```bash
# 发布测试 GPS 数据
ros2 topic pub /gps/fix sensor_msgs/msg/NavSatFix \
  "{latitude: 22.5, longitude: 113.9, status: {status: 0}}" --once

# 发布测试模式切换
ros2 topic pub /system_mode std_msgs/msg/String "{data: 'follow'}" --once

# 发布测试手势指令
ros2 topic pub /gesture_cmd std_msgs/msg/String "{data: 'stop'}" --once
```

### 6.4 提交前自检清单

每次提交 PR 前，请逐项确认:

- [ ] `python3 -m py_compile` 通过（所有修改的 .py 文件）
- [ ] `colcon build --packages-select your_package` 编译通过
- [ ] 没有修改其他人负责包的文件
- [ ] 没有硬编码 IP、密码、绝对路径
- [ ] 话题名和消息类型没有变化（如有变化，已提前通知团队）
- [ ] 新节点已在 `setup.py` 中注册
- [ ] 新增的 launch 文件已在 `setup.py` 的 `data_files` 中包含
- [ ] 代码有基本注释，关键逻辑有说明

---

## 7. 项目负责人集成流程

### 7.1 日常 Review

- 每周 1-2 次审核并合并 PR
- 重点检查:
  - 是否影响其他包的接口
  - 是否引入硬编码
  - 是否修改了不属于自己的文件
  - commit message 是否规范

### 7.2 集成验证

```bash
# 1. 拉取最新代码
cd ~/golf_ws/src
git pull origin main

# 2. 全量编译
cd ~/golf_ws
colcon build --symlink-install

# 3. 同步到 WheelTec 实车
rsync -avz --delete \
    --exclude='build/' --exclude='install/' --exclude='log/' \
    --exclude='.git/' --exclude='__pycache__/' --exclude='*.pyc' \
    --exclude='*.engine' --exclude='third_party/lx_camera_ros/' \
    ~/golf_ws/src/ wheeltec:~/golf_ws/src/

# 4. 在 WheelTec 上编译
ssh wheeltec "cd ~/golf_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# 5. 远程验证话题是否正常
ssh wheeltec "source ~/golf_ws/install/setup.bash && ros2 topic list"
ssh wheeltec "source ~/golf_ws/install/setup.bash && ros2 topic hz /gps/fix"
```

### 7.3 实车测试

- 每 1-2 周进行一次实车联调
- 测试前确保:
  - 所有 PR 已合并，代码已同步
  - WheelTec 上编译通过
  - GPS 天线安装正确（ANT1 前，ANT2 后）
- 测试后:
  - 录制 rosbag 供团队回放调试
  - 发现的问题在 GitHub 开 Issue，指定负责人

### 7.4 任务分配（Issue 驱动开发）

> **铁律：工程师不得自行决定优化方向。** 所有开发任务由项目负责人通过 Issue 分配，包含明确的目标和验收标准。

**Issue 创建时会自动填充模板**（`.github/ISSUE_TEMPLATE/task.md`），必填字段：

| 字段 | 说明 |
|------|------|
| 目标 | 要解决什么问题 |
| 验收标准 | 量化指标（如"检测率 ≥ 80%"、"转弯延迟 < 0.3s"） |
| 约束 | 不能改什么（如"不能改话题接口"） |
| rosbag 参考 | 用哪个 bag 验证 |

**Issue 示例**:
```
标题: [tune][perception] 逆光场景人体检测率提升
目标: 当前逆光下 YOLO 检测率 ~50%，目标提升到 ≥ 80%
验收标准: 使用 follow_bag_20260408 回放，逆光帧段检测率 ≥ 80%
约束: 不能降低正常光照下的检测率；不能修改话题接口
rosbag: follow_bag_20260408（共享网盘链接）
负责人: @图像识别工程师
```

**任务生命周期**:
```
项目负责人开 Issue（附目标+验收标准+rosbag）
    → 工程师认领，开 feature 分支
    → 开发 + rosbag 回放验证
    → 提 PR（关联 Issue，附验证数据）
    → 项目负责人 review + merge
    → 实车测试验收
    → 通过：关闭 Issue | 不通过：追加评论，工程师继续修
```

### 7.5 文档维护

> merge PR 后，项目负责人负责检查并更新以下文档（保持文档与代码同步）：

| 变更类型 | 需要更新的文档 |
|---------|---------------|
| 参数调整 | 对应包 README.md 参数表 |
| 话题接口变更 | `INTERFACE_CONTRACT.md` 话题契约表 |
| 新增/删除节点 | `ARCHITECTURE.md` 节点表 + 数据流图 |
| 数据流变化 | `ARCHITECTURE.md` 数据流图 |
| 新增启动参数 | 对应 launch 文件注释 + 包 README |

**文档更新必须与代码变更在同一轮集成中完成，不允许延后。**

### 7.6 版本管理

- 每次实车测试通过后打 tag: `git tag v8.3.x`
- 补丁号每次 +1（v8.3.0 → v8.3.1 → v8.3.2）
- 阶段性里程碑次版本号 +1（v8.3.x → v8.4.0）

---

## 8. 常见问题

### Q: 编译报错 `package not found`

```bash
# 确保 source 了 ROS2 环境
source /opt/ros/humble/setup.bash
source ~/golf_ws/install/setup.bash

# 如果依赖其他包，先编译依赖
colcon build --symlink-install
```

### Q: 新增的节点 `ros2 run` 找不到

检查两件事:
1. `setup.py` 的 `entry_points` 中是否注册了节点
2. 是否重新编译: `colcon build --symlink-install --packages-select your_package`

### Q: 话题没有数据

```bash
# 确认话题是否存在
ros2 topic list | grep your_topic

# 确认话题频率
ros2 topic hz /your_topic

# 确认消息类型是否匹配
ros2 topic info /your_topic
```

### Q: 我需要修改其他包的接口怎么办

1. 先在群里/Issue 中提出需求
2. 与接口两端的负责人对齐方案
3. 由**接口提供方**的负责人修改并提交 PR
4. 所有相关方同步更新自己的订阅代码

### Q: transforms3d 报 `np.float` 错误

numpy 2.x 移除了 `np.float`，需修复:
```bash
# 找到并替换
python3 -c "import transforms3d; print(transforms3d.__file__)"
# 将文件中的 np.float 替换为 np.float64
```

### Q: Jetson 上 torch 安装失败

- 禁止使用 `pip install torch`（x86 版本，Jetson 无法运行）
- 必须使用 NVIDIA 官方提供的 Jetson 专用 wheel
- 参考: https://forums.developer.nvidia.com/t/pytorch-for-jetson/

---

## 附录: 工作空间结构速查

```
~/golf_ws/src/
├── golf_bringup/                    # [项目负责人] 启动与配置
│   ├── launch/
│   │   ├── bringup_launch.py        # 全栈启动
│   │   ├── sensors.launch.py        # 传感器层
│   │   ├── perception.launch.py     # 感知层
│   │   ├── follow.launch.py         # 跟随模式
│   │   ├── navigation.launch.py     # 导航模式
│   │   ├── minimal_nav2.launch.py   # Nav2 最小启动
│   │   ├── slam.launch.py           # SLAM 建图
│   │   ├── s11_tf.launch.py         # S11 相机 TF
│   │   └── depth_scan.launch.py     # 深度图转 scan
│   ├── config/
│   │   ├── nav2_params.yaml         # Nav2 参数
│   │   ├── ekf_gps.yaml             # EKF GPS 融合参数
│   │   ├── collision_monitor.yaml   # 碰撞监测参数
│   │   └── slam_toolbox.yaml        # SLAM 参数
│   └── urdf/                        # 机器人模型描述
│
├── golf_perception/                 # [图像识别工程师] 感知
│   └── golf_perception/
│       ├── detection_node.py        # YOLO 检测（备用本地推理）
│       ├── gesture_node.py          # 手势识别 → /gesture_cmd
│       ├── camera_info_fix.py       # 相机内参修复
│       └── image_resizer.py         # 调试图像缩放
│
├── golf_navigation/                 # [规控工程师] 导航控制
│   └── golf_navigation/
│       ├── mode_manager_node.py     # 系统模式管理（idle/follow/navigate）
│       ├── follow_target_publisher.py  # 跟随目标 → 速度指令
│       ├── gps_waypoint_follower.py # GPS-PID 路点导航
│       ├── lidar_emergency_stop.py  # LiDAR 急停中继
│       ├── lock_manager_node.py     # 目标锁定管理
│       └── summon_service.py        # 召唤服务
│
├── golf_mapping/                    # [地图+传感器工程师] 建图
│   └── golf_mapping/
│       ├── gps_path_recorder.py     # GPS 路点记录（加权平均）
│       ├── imu_ned_to_enu.py        # IMU NED→ENU + G90 航向融合
│       └── test_logger.py           # 测试数据记录
│
└── golf_communication/              # [项目负责人] 通信
    └── golf_communication/
        └── mqtt_bridge_node.py      # MQTT ↔ ROS2 桥接
```
