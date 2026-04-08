# golf_communication

高尔夫球车通信桥接包 -- 通过 MQTT 协议桥接 ROS2 话题与手机 APP，实现 4G 远程控制与状态监控。

## 节点列表

| 节点名 | 功能 | 订阅话题 | 发布话题 |
|--------|------|----------|----------|
| `mqtt_bridge` | ROS2 ↔ MQTT 双向桥接，支持车辆状态上报和 APP 远程命令下发 | `/system_mode` (String), `/follow_state` (String), `/path_graph/stats` (String), `/gps/fix` (NavSatFix), `/summon_result` (String), `/nav_complete` (String) | `/summon_request` (String), `/mark_waypoint_label` (String), `/nav_trigger` (String) |

## MQTT 话题映射

### 车 → APP (ROS2 订阅 → MQTT 发布)

| ROS2 话题 | MQTT 话题 | QoS | 说明 |
|-----------|-----------|-----|------|
| `/system_mode` | `golf/{vehicle_id}/status/mode` | 0 | 系统模式 (following/navigation/e_stop) |
| `/gps/fix` | `golf/{vehicle_id}/status/gps` | 0 | GPS 位置 (JSON: lat, lon, alt, status) |
| `/path_graph/stats` | `golf/{vehicle_id}/status/path_stats` | 0 | 路点图统计信息 |
| `/follow_state` | `golf/{vehicle_id}/status/follow` | 0 | 跟随状态 (idle/tracking) |
| `/summon_result` | `golf/{vehicle_id}/status/summon_result` | 1 | 召唤结果 (JSON: status, detail) |
| `/nav_complete` | `golf/{vehicle_id}/status/nav_complete` | 1 | 导航完成通知 |
| (定时 5s) | `golf/{vehicle_id}/status/heartbeat` | 0 | 车辆在线心跳 |

### APP → 车 (MQTT 订阅 → ROS2 发布)

| MQTT 话题 | ROS2 话题 | 说明 |
|-----------|-----------|------|
| `golf/{vehicle_id}/cmd/summon` | `/summon_request` | 召唤请求 (JSON: {target} 或 {lat, lon}) |
| `golf/{vehicle_id}/cmd/mark` | `/mark_waypoint_label` | 标记路点 (纯文本路点名) |
| `golf/{vehicle_id}/cmd/mode` | `/nav_trigger` | 模式切换 (e_stop/cancel/resume/路点名) |

## 参数说明

### mqtt_bridge

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `mqtt_host` | string | `localhost` | MQTT Broker 地址 |
| `mqtt_port` | int | 1883 | MQTT Broker 端口 |
| `mqtt_keepalive` | int | 60 | MQTT 连接保活间隔 (秒) |
| `mqtt_username` | string | (空) | MQTT 认证用户名 |
| `mqtt_password` | string | (空) | MQTT 认证密码 (也可通过环境变量 MQTT_PASSWORD 设置) |
| `vehicle_id` | string | `cart_01` | 车辆 ID，用于 MQTT 话题前缀隔离 |

## 编译方法

```bash
cd ~/golf_ws
colcon build --packages-select golf_communication --symlink-install
source install/setup.bash
```

**额外依赖：**

```bash
pip3 install paho-mqtt
```

## 测试方法

### 单节点启动验证

```bash
# 启动 MQTT bridge（需要 MQTT Broker 已运行）
ros2 run golf_communication mqtt_bridge --ros-args \
  -p mqtt_host:=localhost \
  -p vehicle_id:=cart_01

# 验证 MQTT 连接状态（查看日志输出 "MQTT connected"）
```

### MQTT Broker 端验证

```bash
# 监听所有车辆状态话题
mosquitto_sub -h localhost -t 'golf/cart_01/status/#' -v

# 发送召唤命令
mosquitto_pub -h localhost -t 'golf/cart_01/cmd/summon' -m '{"target": "3号洞"}'

# 发送 GPS 坐标召唤
mosquitto_pub -h localhost -t 'golf/cart_01/cmd/summon' -m '{"lat": 22.66, "lon": 114.22}'

# 发送急停
mosquitto_pub -h localhost -t 'golf/cart_01/cmd/mode' -m 'e_stop'

# 恢复跟随
mosquitto_pub -h localhost -t 'golf/cart_01/cmd/mode' -m 'resume'

# 标记路点
mosquitto_pub -h localhost -t 'golf/cart_01/cmd/mark' -m '发球台'
```

### ROS2 端验证

```bash
# 模拟 GPS 数据确认 MQTT 转发
ros2 topic echo /summon_request
ros2 topic echo /nav_trigger
ros2 topic echo /mark_waypoint_label
```

## 注意事项

1. **线程安全**：paho-mqtt 回调在后台线程执行，不直接调用 ROS2 API。所有 MQTT 消息通过 `queue.Queue` 传递到 ROS2 定时器 (20Hz) 消费，保证线程安全。
2. **paho-mqtt 兼容性**：兼容 paho-mqtt 1.x 和 2.x 两个版本的 CallbackAPIVersion 接口。
3. **密码安全**：MQTT 密码优先从环境变量 `MQTT_PASSWORD` 读取，避免在参数文件中明文存储。
4. **重连机制**：使用 `connect_async` + `loop_start` 异步连接，paho 内置自动重连。client_id 带随机后缀防止重连风暴。
5. **QoS 分级**：普通状态数据 (mode, gps, follow) 使用 QoS 0 (最多一次)；关键通知 (summon_result, nav_complete) 使用 QoS 1 (至少一次)，确保 APP 收到。
6. **心跳机制**：每 5 秒发送 heartbeat 消息，APP 端可据此判断车辆是否在线。
7. **MQTT Broker**：需要在可达的服务器上运行 Mosquitto 或其他 MQTT Broker。通过 4G 网络连接时注意 Broker 的公网可达性和防火墙配置。
