# golf_navigation

高尔夫球车规划控制包 -- 模式管理、人员跟随、GPS 路点导航（v8.3 脉冲转向）、APP 召唤调度。

## 模块职责

本包是整套系统的**规划与控制部门**：消费 `golf_mapping` 产出的 `path_graph.json` 路点图，结合实时 GPS/航向/LiDAR 生成底盘速度指令。

- **输入**：`data/production/path_graph.json`（由 golf_mapping 产出）+ `/gps/fix` + `/heading_deg` + `/scan` + `/yolo/tracking` + `/gesture_cmd` + `/nav_trigger`
- **输出**：`/cmd_vel`（跟随）、`/cmd_vel_nav`（导航，经 launch 重映射到 `/cmd_vel`）、`/system_mode`、`/nav_complete`、`/summon_result`
- **上游依赖**：golf_mapping 的 `path_graph.json`、golf_perception 的 `/yolo/tracking` 和 `/gesture_cmd`
- **责任人**：规划控制工程师（独立负责本包全部 5 个节点）
- **禁区**：不得修改 `golf_mapping/` 和 `golf_perception/` 任何文件；严禁直接写 `data/production/path_graph.json`（只读消费）；跨包通信严格走话题契约，见 `docs/INTERFACE_CONTRACT.md`

## 节点列表

| 节点名 | 功能 | 订阅话题 | 发布话题 |
|--------|------|----------|----------|
| `mode_manager` | FOLLOWING / NAVIGATION / E_STOP 三状态机，统一管理系统模式 | `/follow_state` (String), `/gesture_cmd` (String), `/nav_trigger` (String), `/joy` (Joy), `/nav_complete` (String) | `/system_mode` (String), `/cmd_vel` (Twist) |
| `follow_target_publisher` | PID / MPPI 双模式人员跟随控制器，根据 YOLO 检测 + 深度图计算目标位置后输出速度指令 | `/follow_state` (String), `/system_mode` (String), `/lx_camera_node/LxCamera_Depth` (Image), `/locked_target` (Detection) | `/cmd_vel` (Twist, PID 模式) 或 FollowPath action (MPPI 模式) |
| `lock_manager` | 自动锁定画面中最大人体 bbox 并持续跟踪，手势暂停/恢复 | `/gesture_cmd` (String), `/yolo/tracking` (DetectionArray) | `/follow_state` (String), `/locked_target` (Detection) |
| `gps_waypoint_follower` | Dijkstra 路径规划 + v8.3 脉冲转向 GPS 路点跟随（内置 LiDAR ±75° 避墙脉冲，亦支持 MPPI / Pure Pursuit 备用模式） | `/gps/fix` (NavSatFix), `/heading_deg` (Float32), `/scan` (LaserScan), `/nav_trigger` (String), `/system_mode` (String), `/nav_blocked` (String) | `/cmd_vel_nav` (Twist, 脉冲模式) 或 FollowPath action (MPPI 模式), `/nav_complete` (String) |
| `summon_service` | APP 召唤请求分发器，支持球洞名和 GPS 坐标两种召唤方式 | `/summon_request` (String), `/nav_complete` (String) | `/nav_trigger` (String), `/summon_result` (String) |

## 参数说明

### mode_manager

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `lost_estop_timeout` | float | 15.0 | 目标丢失超时进入 E_STOP (秒) |
| `estop_joy_button` | int | 0 | 手柄急停按钮索引 |
| `resume_joy_button` | int | 1 | 手柄恢复按钮索引 |

### follow_target_publisher

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `target_distance` | int | 600 | 目标跟随距离 (mm) |
| `max_speed` | float | 0.3 | 最大线速度 (m/s) |
| `angular_kp` | float | 1.2 | 角速度 P 增益 |
| `angular_ki` | float | 0.0 | 角速度 I 增益 |
| `angular_kd` | float | 0.005 | 角速度 D 增益 |
| `linear_kp` | float | 0.2 | 线速度 P 增益 |
| `linear_ki` | float | 0.0 | 线速度 I 增益 |
| `linear_kd` | float | 0.0 | 线速度 D 增益 |
| `out_of_range` | int | 2000 | 超出范围停车距离 (mm) |
| `min_depth` | int | 300 | 最小有效深度 (mm) |
| `tan_half_fov` | float | 1.7321 | S11 水平半视场角正切值 (120 FOV) |
| `use_mppi` | bool | false | 启用 MPPI 避障跟随模式 |

### lock_manager

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `lost_timeout` | float | 10.0 | 目标丢失超时回到 idle (秒) |

### gps_waypoint_follower（v8.3.2 脉冲模式）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `data_file` | string | `~/golf_ws/data/production/path_graph.json` | 路径图 JSON（由 `mode` launch 参数切换 production/test） |
| `arrival_tolerance` | float | 5.0 | 到达判定距离 (m) |
| `max_speed` | float | 0.5 | 最大线速度 (m/s)，launch 里通常覆盖为 0.6 |
| `slowdown_distance` | float | 10.0 | 开始减速距离 (m) |
| `waypoint_skip` | int | 2 | 直线段跳跃路点数 |
| `turn_threshold` | float | 30.0 | 拐点判定角度 (度) |
| `bearing_dead_zone` | float | 20.0 | 脉冲触发方位角死区 (度) |
| `gps_smooth_window` | int | 3 | GPS 滑动平均窗口 |
| `lin_kp` | float | 0.1 | 线速度 P 增益（`linear_x = min(max_speed, dist*lin_kp)`） |
| `max_angular` | float | 0.15 | 安全上限角速度 (rad/s) |
| `gps_timeout` | float | 3.0 | GPS 信号超时 (秒) |
| `pulse_nav_w` | float | 0.12 | 方向修正脉冲角速度 (rad/s) |
| `pulse_nav_on` | float | 0.5 | 方向脉冲触发持续 (s) |
| `pulse_nav_off` | float | 0.5 | 方向脉冲冷却持续 (s) |
| `pulse_wall_w` | float | 0.1 | 避墙脉冲角速度 (rad/s) |
| `pulse_wall_on` | float | 0.5 | 避墙脉冲触发持续 (s) |
| `pulse_wall_off` | float | 0.5 | 避墙脉冲冷却持续 (s) |
| `wall_threshold` | float | 3.0 | 侧边护栏触发距离 (m)，检测范围前方 ±75° 扇区 |
| `cmd_vel_topic` | string | `/cmd_vel_nav` | 速度指令输出话题（launch 里直发 `/cmd_vel`） |
| `use_mppi` | bool | false | 启用 MPPI 样条避障模式（备用，脉冲模式不用） |
| `use_pure_pursuit` | bool | false | 启用 Pure Pursuit 模式（备用） |
| `lookahead_distance` | float | 10.0 | Pure Pursuit 前瞻距离 (m) |
| `spline_spacing` | float | 2.0 | MPPI 样条插值点间距 (m) |

### summon_service

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `data_file` | string | `~/golf_ws/data/production/path_graph.json` | 路径图 JSON（由 `mode` launch 参数切换 production/test） |

## 编译方法

```bash
cd ~/golf_ws
colcon build --packages-select golf_navigation --symlink-install
source install/setup.bash
```

## 测试方法

三层测试体系：L1 本地纯回放 → L2 远程 WheelTec bag 回放 → L3 实车户外（仅负责人执行）。工程师应能独立走完 L1 + L2，L3 提交代码后由负责人现场验证。

### L1 本地 bag 回放（engineer 日常开发）

从负责人处拿到 `nav_bag_*.tar.gz`（含 bag 和基线 `path_graph.json`）后，在本地 ROS2 Humble 环境完成。**所有写操作必须指向 `data/test/`，严禁写 `data/production/`**。

```bash
# 把基线路点图放到 test 目录（只读消费，不覆盖生产数据）
mkdir -p ~/golf_ws/data/test
cp nav_bag_*/baseline_path_graph.json ~/golf_ws/data/test/path_graph.json

# 回放 bag
ros2 bag play nav_bag_*/ --clock --loop

# 另开终端：跑 follower 读 test 路点图，输出到备用话题（不触发底盘）
ros2 run golf_navigation gps_waypoint_follower \
    --ros-args -p use_sim_time:=true \
    -p data_file:=$HOME/golf_ws/data/test/path_graph.json \
    -p cmd_vel_topic:=/cmd_vel_nav_test

# 观察决策输出
ros2 topic echo /cmd_vel_nav_test

# 触发一次导航
ros2 topic pub /nav_trigger std_msgs/msg/String "data: '3号洞'" --once
```

**单节点启动验证**（不依赖 bag，仅用于代码改动后的冒烟测试）：

```bash
# 模式管理器
ros2 run golf_navigation mode_manager
ros2 topic echo /system_mode

# 跟随控制器（需感知话题）
ros2 run golf_navigation follow_target_publisher
ros2 topic hz /cmd_vel

# 目标锁定
ros2 run golf_navigation lock_manager
ros2 topic echo /follow_state

# 召唤服务
ros2 run golf_navigation summon_service \
    --ros-args -p data_file:=$HOME/golf_ws/data/test/path_graph.json
ros2 topic pub /summon_request std_msgs/msg/String "data: '{\"target\": \"3号洞\"}'" --once
```

### L2 远程 WheelTec bag 回放（engineer 需 SSH 权限）

通过 VPS 跳板 SSH 到 WheelTec 上 rsync 代码、编译、回放 bag 验证新代码在目标硬件上编译运行正常。**禁止触碰实车驾驶、`data/production/` 目录和任何实际 `/cmd_vel` 发布。**

```bash
# 代码同步到 WheelTec
rsync -avz --exclude='build/' --exclude='install/' --exclude='third_party/lx_camera_ros/' \
    ~/golf_ws/src/golf_navigation/ wheeltec:~/golf_ws/src/golf_navigation/

# WheelTec 上编译
ssh wheeltec 'cd ~/golf_ws && colcon build --packages-select golf_navigation --symlink-install'

# WheelTec 上回放 bag + test 模式 follower
ssh wheeltec 'bash -lc "source ~/golf_ws/install/setup.bash && \
    ros2 bag play ~/golf_ws/bags/nav_bag_YYYYMMDD_HHMMSS/ --clock &
    ros2 run golf_navigation gps_waypoint_follower \
        --ros-args -p use_sim_time:=true \
        -p data_file:=\$HOME/golf_ws/data/test/path_graph.json \
        -p cmd_vel_topic:=/cmd_vel_nav_test"'
```

### L3 实车户外测试（仅负责人执行）

engineer 通过 PR 提交改动，负责人 merge 后在车上执行：

```bash
# 负责人在 WheelTec 上启动完整系统（生产模式）
bash ~/golf_ws/start_all.sh

# 或手动启动
ros2 launch golf_bringup follow.launch.py mode:=production
```

### 模式切换测试（任一层级可用）

```bash
# 触发导航
ros2 topic pub /nav_trigger std_msgs/msg/String "data: '3号洞'" --once

# 远程急停
ros2 topic pub /nav_trigger std_msgs/msg/String "data: 'e_stop'" --once

# 恢复跟随
ros2 topic pub /nav_trigger std_msgs/msg/String "data: 'resume'" --once

# 取消导航
ros2 topic pub /nav_trigger std_msgs/msg/String "data: 'cancel'" --once
```

### 验收标准

- **脉冲状态机正确性**：L1 bag 回放下观察 `/cmd_vel_nav_test`，直线段 `angular_z ≈ 0`，方位角误差超出 `bearing_dead_zone` 时脉冲触发且方向正确（error>0 → 右转 w<0）
- **避墙触发**：LiDAR 侧方距离 < `wall_threshold` 时产生对应方向的避墙脉冲，冷却期间不再触发
- **Dijkstra 路径**：`/nav_trigger` 触发后规划路径点数与 path_graph 拓扑一致，起点选择 = 最近路点
- **话题契约**：未修改 `/cmd_vel_nav` / `/nav_complete` / `/summon_result` 的消息类型和频率，见 `docs/INTERFACE_CONTRACT.md`
- **生产数据保护**：整个测试过程 `data/production/path_graph.json` 的文件修改时间戳**不变**

## 交付物

engineer 本地或 L2 测试完成后，在 PR 中提供以下产物：

| 文件 | 位置 | 用途 |
|------|------|------|
| 改动代码 | `src/golf_navigation/golf_navigation/*.py` | 明确说明只改了哪些节点的哪些函数 |
| bag 回放截图 / 日志 | PR 描述 | `/cmd_vel_nav_test` 话题的时间序列、脉冲触发点可视化 |
| PR 描述 | GitHub | "改了什么 / 为什么改 / 效果对比 / 未触及的节点清单" |

**禁止物**：任何对 `golf_mapping/` 或 `golf_perception/` 的修改、任何对 `data/production/` 的写操作、任何对节点话题接口（消息类型/名称/频率）的改动、任何向真实 `/cmd_vel` 话题发布的测试（底盘会动，有安全风险）。

## 注意事项

0. **生产/测试数据隔离铁律**：`gps_waypoint_follower` 和 `summon_service` 默认读 `data/production/path_graph.json`，测试必须通过 `mode:=test` launch 参数或显式 `-p data_file:=...` 隔离到 `data/test/`。生产路点图是实车导航的唯一真实来源，一旦污染会导致召唤指到错误坐标，严禁在测试链路中写入。
1. **PID 参数来自 WheelTec 官方**：`follow_target_publisher` 中的 simplePID 类和参数完整复制自 WheelTec `simple_follower_ros2/visualFollower.py`，不要随意修改。
2. **v8.3 脉冲转向导航**：`gps_waypoint_follower` 使用脉冲状态机（0=直行 / 1=方向修正脉冲 / 2=方向修正冷却 / 3=避墙脉冲 / 4=避墙冷却）。方位角误差超过 `bearing_dead_zone` 时发射一个 `pulse_nav_on` 秒的脉冲，之后强制 `pulse_nav_off` 秒冷却，直线段保持 `angular_z=0`。参数调整须极其保守，每次只改一个，且禁止恢复持续 PID 转向。
3. **LiDAR 避墙**：扫描前方 ±75° 扇区，侧方距离 < `wall_threshold` 时触发避墙脉冲（远离墙一侧）。脉冲结束后同样进入冷却，不与方向修正脉冲叠加。
4. **cmd_vel 链路**：导航模式 `gps_waypoint_follower` 直发 `/cmd_vel`（launch 里 `cmd_vel_topic:=/cmd_vel`，不经过任何中继节点）；跟随模式 `follow_target_publisher → /cmd_vel`。已不再使用 `lidar_emergency_stop` 节点，避墙能力完全内置于 follower 脉冲状态机。
5. **MPPI / Pure Pursuit 备用模式**：`use_mppi=true` 需要 Nav2 的 `controller_server` 已启动；`use_pure_pursuit=true` 切换到前瞻控制。两者均为回退方案，当前 v8.3 实车使用纯脉冲模式，不默认启用。
6. **路径图格式**：`path_graph.json` 包含 `waypoints`（路点 ID → lat/lon/label）和 `edges`（双向边 + 距离），由 `golf_mapping` 包的 `gps_path_recorder` 生成，本包只读消费。
7. **G90 双天线航向**：`gps_waypoint_follower` 依赖 `/heading_deg` (Float32) 提供罗盘航向，航向丢失超过 `gps_timeout` 秒时停车等待。
