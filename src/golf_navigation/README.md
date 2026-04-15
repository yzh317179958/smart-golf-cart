# golf_navigation

高尔夫球车规划控制包：模式管理、人员跟随、GPS 路点导航（v8.3 脉冲转向）。

## 节点列表

| 节点名 | 功能 | 订阅话题 | 发布话题 |
|--------|------|----------|----------|
| `mode_manager` | FOLLOWING / NAVIGATION / E_STOP 三状态机，统一管理系统模式 | `/follow_state`, `/gesture_cmd`, `/nav_trigger`, `/joy`, `/nav_complete` | `/system_mode`, `/cmd_vel` |
| `follow_target_publisher` | PID 人员跟随控制器，根据 YOLO 检测 + 深度图计算目标位置后输出速度指令 | `/follow_state`, `/system_mode`, `/lx_camera_node/LxCamera_Depth`, `/locked_target` | `/cmd_vel` |
| `lock_manager` | 自动锁定画面中最大人体 bbox 并持续跟踪，手势暂停/恢复 | `/gesture_cmd`, `/yolo/tracking` | `/follow_state`, `/locked_target` |
| `gps_waypoint_follower` | Dijkstra 路径规划 + v8.3 脉冲转向 GPS 路点跟随，内置 LiDAR ±75° 避墙脉冲 | `/gps/fix`, `/heading_deg`, `/scan`, `/nav_trigger`, `/system_mode`, `/nav_blocked` | `/cmd_vel_nav`（launch 重映射为 `/cmd_vel`），`/nav_complete` |

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
| `tan_half_fov` | float | 1.7321 | S11 水平半视场角正切值 (120° FOV) |

### lock_manager

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `lost_timeout` | float | 10.0 | 目标丢失超时回到 idle (秒) |

### gps_waypoint_follower (v8.3.2 脉冲模式)

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
| `lin_kp` | float | 0.1 | 线速度 P 增益 |
| `max_angular` | float | 0.15 | 安全上限角速度 (rad/s) |
| `gps_timeout` | float | 3.0 | GPS 信号超时 (秒) |
| `pulse_nav_w` | float | 0.12 | 方向修正脉冲角速度 (rad/s) |
| `pulse_nav_on` | float | 0.5 | 方向脉冲触发持续 (s) |
| `pulse_nav_off` | float | 0.5 | 方向脉冲冷却持续 (s) |
| `pulse_wall_w` | float | 0.1 | 避墙脉冲角速度 (rad/s) |
| `pulse_wall_on` | float | 0.5 | 避墙脉冲触发持续 (s) |
| `pulse_wall_off` | float | 0.5 | 避墙脉冲冷却持续 (s) |
| `wall_threshold` | float | 3.0 | 侧边护栏触发距离 (m)，检测前方 ±75° 扇区 |
| `cmd_vel_topic` | string | `/cmd_vel_nav` | 速度指令输出话题（launch 里直发 `/cmd_vel`） |

## 上下游接口

- **上游**：
  - `golf_mapping/gps_path_recorder` 产出 `path_graph.json`，本包只读消费
  - `golf_perception/detection_node` 发布 `/yolo/tracking`
  - `golf_perception/gesture_node` 发布 `/gesture_cmd`
  - `golf_communication/summon_service` 发布 `/nav_trigger` 触发召唤导航
  - `wheeltec_gps_driver` 提供 `/gps/fix` 和 `/heading_deg`
  - N10P LiDAR 提供 `/scan`
  - 底盘手柄提供 `/joy`
- **下游**：
  - WheelTec 底盘驱动订阅 `/cmd_vel`
  - `golf_communication/mqtt_bridge` 订阅 `/system_mode`、`/nav_complete` 转发到 APP
  - `golf_communication/summon_service` 订阅 `/nav_complete` 生成 `/summon_result`

## 编译

```bash
cd ~/golf_ws
colcon build --packages-select golf_navigation --symlink-install
source install/setup.bash
```

## 启动

通过 `golf_bringup` 统一 launch（推荐）：

```bash
# production（实车）
ros2 launch golf_bringup follow.launch.py

# test（回放/测试路点写到 data/test/）
ros2 launch golf_bringup follow.launch.py mode:=test
```

单节点启动：

```bash
ros2 run golf_navigation mode_manager
ros2 run golf_navigation lock_manager
ros2 run golf_navigation follow_target_publisher
ros2 run golf_navigation gps_waypoint_follower
```

触发导航：

```bash
# 按球洞名导航
ros2 topic pub /nav_trigger std_msgs/msg/String "data: '3号洞'" --once

# 急停 / 恢复 / 取消
ros2 topic pub /nav_trigger std_msgs/msg/String "data: 'e_stop'" --once
ros2 topic pub /nav_trigger std_msgs/msg/String "data: 'resume'" --once
ros2 topic pub /nav_trigger std_msgs/msg/String "data: 'cancel'" --once
```

## 注意事项

1. **生产/测试数据隔离**：`gps_waypoint_follower` 默认读 `data/production/path_graph.json`，测试必须通过 `mode:=test` 或显式 `-p data_file:=...` 隔离到 `data/test/`。
2. **v8.3 脉冲转向**：`gps_waypoint_follower` 使用脉冲状态机（0 直行 / 1 方向脉冲 / 2 方向冷却 / 3 避墙脉冲 / 4 避墙冷却），方位角误差超过 `bearing_dead_zone` 时发射 `pulse_nav_on` 秒脉冲后强制冷却，直线段保持 `angular_z=0`。禁止恢复持续 PID 转向。
3. **cmd_vel 链路**：导航 `gps_waypoint_follower` 直发 `/cmd_vel`（launch 里 `cmd_vel_topic:=/cmd_vel`），跟随 `follow_target_publisher → /cmd_vel`。两者都不经过任何中继节点，避墙能力完全内置于 follower 脉冲状态机。
4. **WheelTec 跟随 PID 原版**：`follow_target_publisher` 的 simplePID 类和默认参数完整复制自 WheelTec `simple_follower_ros2/visualFollower.py`，不要随意修改。
5. **G90 双天线航向**：`gps_waypoint_follower` 依赖 `/heading_deg` 提供罗盘航向，航向丢失超过 `gps_timeout` 秒时停车等待。
