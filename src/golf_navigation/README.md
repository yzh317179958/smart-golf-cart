# golf_navigation

高尔夫球车规划控制包 -- 模式管理、人员跟随、GPS 路点导航、LiDAR 安全防护、APP 召唤调度。

## 节点列表

| 节点名 | 功能 | 订阅话题 | 发布话题 |
|--------|------|----------|----------|
| `mode_manager` | FOLLOWING / NAVIGATION / E_STOP 三状态机，统一管理系统模式 | `/follow_state` (String), `/gesture_cmd` (String), `/nav_trigger` (String), `/joy` (Joy), `/nav_complete` (String) | `/system_mode` (String), `/cmd_vel` (Twist) |
| `follow_target_publisher` | PID / MPPI 双模式人员跟随控制器，根据 YOLO 检测 + 深度图计算目标位置后输出速度指令 | `/follow_state` (String), `/system_mode` (String), `/lx_camera_node/LxCamera_Depth` (Image), `/locked_target` (Detection) | `/cmd_vel` (Twist, PID 模式) 或 FollowPath action (MPPI 模式) |
| `lock_manager` | 自动锁定画面中最大人体 bbox 并持续跟踪，手势暂停/恢复 | `/gesture_cmd` (String), `/yolo/tracking` (DetectionArray) | `/follow_state` (String), `/locked_target` (Detection) |
| `gps_waypoint_follower` | Dijkstra 路径规划 + GPS-PID 逐点导航（支持 MPPI 样条避障模式和 Pure Pursuit 模式） | `/gps/fix` (NavSatFix), `/heading_deg` (Float32), `/scan` (LaserScan), `/nav_trigger` (String), `/system_mode` (String), `/nav_blocked` (String) | `/cmd_vel_nav` (Twist, PID 模式) 或 FollowPath action (MPPI 模式), `/nav_complete` (String) |
| `lidar_emergency_stop` | LiDAR 前方障碍物检测安全门控，对 `/cmd_vel_nav` 进行透传/减速/停车处理 | `/scan` (LaserScan), `/cmd_vel_nav` (Twist) | `/cmd_vel` (Twist), `/nav_blocked` (String), `/emergency_stop_state` (String) |
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

### gps_waypoint_follower

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `data_file` | string | `~/golf_ws/data/path_graph.json` | 路径图 JSON 文件路径 |
| `arrival_tolerance` | float | 5.0 | 到达判定距离 (m) |
| `max_speed` | float | 0.5 | 最大线速度 (m/s) |
| `slowdown_distance` | float | 10.0 | 开始减速距离 (m) |
| `waypoint_skip` | int | 2 | 直线段跳跃路点数 |
| `turn_threshold` | float | 30.0 | 拐点判定角度 (度) |
| `bearing_dead_zone` | float | 5.0 | 方位角死区 (度) |
| `gps_smooth_window` | int | 3 | GPS 滑动平均窗口 |
| `angular_smooth` | float | 0.3 | 角速度低通滤波系数 (0~1) |
| `ang_kp` | float | 0.8 | 转向 P 增益 |
| `ang_kd` | float | 0.15 | 转向 D 增益 |
| `lin_kp` | float | 0.1 | 速度 P 增益 |
| `max_angular` | float | 0.8 | 最大角速度 (rad/s) |
| `gps_timeout` | float | 3.0 | GPS 信号超时 (秒) |
| `cmd_vel_topic` | string | `/cmd_vel_nav` | 速度指令输出话题 |
| `use_mppi` | bool | false | 启用 MPPI 样条避障模式 |
| `use_pure_pursuit` | bool | false | 启用 Pure Pursuit 模式 |
| `lookahead_distance` | float | 10.0 | Pure Pursuit 前瞻距离 (m) |
| `spline_spacing` | float | 2.0 | 样条插值点间距 (m) |

### lidar_emergency_stop

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `stop_distance` | float | 2.0 | 停车距离 (m) |
| `slowdown_distance` | float | 3.0 | 减速距离 (m) |
| `slowdown_ratio` | float | 0.5 | 减速比例 |
| `front_angle` | float | 30.0 | 前方检测半角 (度) |
| `clear_time` | float | 2.0 | 障碍消失确认时间 (秒) |
| `block_timeout` | float | 30.0 | 停车超时通知时间 (秒) |
| `cmd_vel_in` | string | `/cmd_vel_nav` | 输入速度话题 |
| `cmd_vel_out` | string | `/cmd_vel` | 输出速度话题 |

### summon_service

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `data_file` | string | `~/golf_ws/data/path_graph.json` | 路径图 JSON 文件路径 |

## 编译方法

```bash
cd ~/golf_ws
colcon build --packages-select golf_navigation --symlink-install
source install/setup.bash
```

## 测试方法

### 单节点启动验证

```bash
# 模式管理器
ros2 run golf_navigation mode_manager
ros2 topic echo /system_mode

# 跟随控制器
ros2 run golf_navigation follow_target_publisher
ros2 topic hz /cmd_vel

# 目标锁定
ros2 run golf_navigation lock_manager
ros2 topic echo /follow_state

# GPS 导航（需要 GPS 数据和路径图）
ros2 run golf_navigation gps_waypoint_follower
ros2 topic pub /nav_trigger std_msgs/msg/String "data: '3号洞'" --once

# LiDAR 急停（需要 /scan 数据）
ros2 run golf_navigation lidar_emergency_stop
ros2 topic echo /emergency_stop_state

# 召唤服务
ros2 run golf_navigation summon_service
ros2 topic pub /summon_request std_msgs/msg/String "data: '{\"target\": \"3号洞\"}'" --once
```

### 模式切换测试

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

### rosbag 回放测试

```bash
# 回放 GPS + LiDAR 数据
ros2 bag play test_bag/ --clock
ros2 run golf_navigation gps_waypoint_follower --ros-args -p use_sim_time:=true
```

## 注意事项

1. **PID 参数来自 WheelTec 官方**：`follow_target_publisher` 中的 simplePID 类和参数完整复制自 WheelTec `simple_follower_ros2/visualFollower.py`，不要随意修改。
2. **GPS-PID 导航防蛇形**：`gps_waypoint_follower` 内置五层防蛇形机制 -- GPS 滑动平均、Laplacian 路径平滑、路点稀疏化、方位角死区、角速度低通滤波。参数调整需保守渐进。
3. **cmd_vel 链路**：导航模式下速度链路为 `gps_waypoint_follower → /cmd_vel_nav → lidar_emergency_stop → /cmd_vel`；跟随模式下为 `follow_target_publisher → /cmd_vel`（不经过急停节点）。
4. **MPPI 模式依赖**：`use_mppi=true` 需要 Nav2 的 `controller_server` 已启动，否则导航会报错失败。
5. **路径图格式**：`path_graph.json` 包含 `waypoints`（路点 ID → lat/lon/label）和 `edges`（双向边 + 距离），由 `golf_mapping` 包的 `gps_path_recorder` 生成。
6. **G90 双天线航向**：`gps_waypoint_follower` 依赖 `/heading_deg` (Float32) 提供罗盘航向，航向丢失时会停车等待。
