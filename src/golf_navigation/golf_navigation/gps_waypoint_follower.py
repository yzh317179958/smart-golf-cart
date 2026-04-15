"""GPS 路点导航器（v8.3 脉冲转向）

流程:
  /nav_trigger → Dijkstra 路径 → 前瞻 +1 方位角 → 死区脉冲修正
  状态机: STRAIGHT → NAV_PULSE → NAV_COOL → STRAIGHT
  侧边护栏: WALL_PULSE → WALL_COOL，优先级高于方向修正

接口:
  输入: /nav_trigger (String)     — 目标名、路点 ID
  输出: /nav_complete (String)    — arrived / failed / blocked / canceled
  阻断: /nav_blocked (String)     — 来自上游阻断信号
"""

import json
import math
import os
import heapq
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Float32
from sensor_msgs.msg import NavSatFix, LaserScan
from geometry_msgs.msg import Twist

_EARTH_R = 6371000.0


def haversine(lat1, lon1, lat2, lon2):
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2.0) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(dlon / 2.0) ** 2)
    return _EARTH_R * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def compass_bearing(lat1, lon1, lat2, lon2):
    """从点1到点2的罗盘方位角（0=北, 90=东, 180=南, 270=西）"""
    dlat = lat2 - lat1
    dlon = (lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2.0))
    bearing = math.degrees(math.atan2(dlon, dlat))
    return bearing % 360.0


def normalize_angle_deg(angle):
    """归一化角度到 [-180, 180]"""
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


class GpsWaypointFollower(Node):

    def __init__(self):
        super().__init__('gps_waypoint_follower')

        # === 参数 ===
        self.declare_parameter('data_file',
                               os.path.expanduser('~/golf_ws/data/production/path_graph.json'))
        self.declare_parameter('arrival_tolerance', 5.0)     # 到达判定 (m)
        self.declare_parameter('max_speed', 0.5)             # 最大线速度 (m/s)
        self.declare_parameter('slowdown_distance', 10.0)    # 开始减速距离 (m)
        self.declare_parameter('waypoint_skip', 2)           # 直线段跳跃数
        self.declare_parameter('turn_threshold', 30.0)       # 拐点判定角度 (°)
        self.declare_parameter('bearing_dead_zone', 20.0)    # 脉冲触发死区 (°)
        self.declare_parameter('gps_smooth_window', 3)       # GPS 平滑窗口
        self.declare_parameter('lin_kp', 0.1)                # 速度 P
        self.declare_parameter('max_angular', 0.15)          # 安全上限角速度 (rad/s)
        self.declare_parameter('gps_timeout', 3.0)           # GPS 超时 (s)
        self.declare_parameter('pulse_nav_w', 0.12)          # 方向修正脉冲角速度 (rad/s)
        self.declare_parameter('pulse_nav_on', 0.5)          # 方向脉冲持续 (s)
        self.declare_parameter('pulse_nav_off', 0.5)         # 方向冷却持续 (s)
        self.declare_parameter('pulse_wall_w', 0.1)          # 避墙脉冲角速度 (rad/s)
        self.declare_parameter('pulse_wall_on', 0.5)         # 避墙脉冲持续 (s)
        self.declare_parameter('pulse_wall_off', 0.5)        # 避墙冷却持续 (s)
        self.declare_parameter('wall_threshold', 3.0)        # 侧边护栏触发距离 (m)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')

        self.data_file = self.get_parameter('data_file').value
        self.arrival_tol = self.get_parameter('arrival_tolerance').value
        self.max_speed = self.get_parameter('max_speed').value
        self.slowdown_dist = self.get_parameter('slowdown_distance').value
        self.wp_skip = self.get_parameter('waypoint_skip').value
        self.turn_thresh = self.get_parameter('turn_threshold').value
        self.dead_zone = self.get_parameter('bearing_dead_zone').value
        self.gps_window = self.get_parameter('gps_smooth_window').value
        self.lin_kp = self.get_parameter('lin_kp').value
        self.max_angular = self.get_parameter('max_angular').value
        self.gps_timeout = self.get_parameter('gps_timeout').value
        self.pulse_nav_w = self.get_parameter('pulse_nav_w').value
        self.pulse_nav_on_ticks = round(self.get_parameter('pulse_nav_on').value / 0.1)
        self.pulse_nav_off_ticks = round(self.get_parameter('pulse_nav_off').value / 0.1)
        self.pulse_wall_w = self.get_parameter('pulse_wall_w').value
        self.pulse_wall_on_ticks = round(self.get_parameter('pulse_wall_on').value / 0.1)
        self.pulse_wall_off_ticks = round(self.get_parameter('pulse_wall_off').value / 0.1)
        self.wall_threshold = self.get_parameter('wall_threshold').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value

        # === 路点图 ===
        self.waypoints = {}
        self.edges = []
        self.adjacency = {}

        # === 导航状态 ===
        self.navigating = False
        self.nav_path = []          # 稀疏化后的路点 ID 列表
        self.nav_index = 0          # 当前目标路点索引
        self.pending_target = None  # 回调写入，定时器消费

        # === 传感器数据 ===
        self.gps_buffer = []        # [(lat, lon, time), ...] 滚动窗口
        self.heading_deg = None     # G90 双天线航向 (°, 罗盘)
        self.heading_time = 0.0     # 航向最后更新时间
        self.heading_buffer = []    # [(deg, time), ...] 航向滚动窗口，用于平均
        self.gps_time = 0.0        # GPS 最后更新时间

        # === 脉冲状态机 ===
        # 0=直行 1=方向脉冲 2=方向冷却 3=避墙脉冲 4=避墙冷却
        self.pulse_state = 0
        self.pulse_ticks = 0
        self.pulse_dir = 0        # +1=左转(w>0), -1=右转(w<0)

        # === 阻断状态（收到 /nav_blocked 时停车通知用户，不自动后退） ===
        self.blocked = False

        # === LiDAR 侧边护栏 ===
        self.lidar_left_dist = float('inf')   # 左侧最近距离
        self.lidar_right_dist = float('inf')  # 右侧最近距离
        self.lidar_front_dist = float('inf')  # 正前方最近距离

        # === 订阅 ===
        gps_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, gps_qos)
        self.create_subscription(NavSatFix, '/gps/fix', self._gps_cb, gps_qos)
        self.create_subscription(Float32, '/heading_deg', self._heading_cb, gps_qos)
        self.create_subscription(String, '/nav_trigger', self._nav_trigger_cb, 10)
        self.create_subscription(String, '/system_mode', self._mode_cb, 10)
        self.create_subscription(String, '/nav_blocked', self._blocked_cb, 10)

        # === 发布 ===
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.nav_complete_pub = self.create_publisher(String, '/nav_complete', 10)

        # === 定时器 ===
        self.create_timer(0.1, self._nav_loop)  # 10Hz 控制循环

        # === 加载路点图 ===
        self._load_graph()

        self.get_logger().info(
            f'GPS 路点导航器启动 | 脉冲式 死区={self.dead_zone}° '
            f'nav_w={self.pulse_nav_w} wall_w={self.pulse_wall_w} | '
            f'路点: {len(self.waypoints)} | 到达容差: {self.arrival_tol}m | '
            f'max_w={self.max_angular}')

    # ================================================================
    #  回调（轻量）
    # ================================================================

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status < 0 or (msg.latitude == 0.0 and msg.longitude == 0.0):
            return
        now = time.monotonic()
        self.gps_buffer.append((msg.latitude, msg.longitude, now))
        # 只保留最近 N 个
        if len(self.gps_buffer) > self.gps_window * 2:
            self.gps_buffer = self.gps_buffer[-self.gps_window:]
        self.gps_time = now

    def _heading_cb(self, msg: Float32):
        self.heading_deg = msg.data % 360.0
        now = time.monotonic()
        self.heading_time = now
        self.heading_buffer.append((self.heading_deg, now))
        # 保留最近 20 个读数 (~2秒 @10Hz)
        if len(self.heading_buffer) > 20:
            self.heading_buffer = self.heading_buffer[-20:]

    def _scan_cb(self, msg: LaserScan):
        """从 /scan 提取前方左/前方右最近距离（前方 ±75° 扇区）"""
        min_front_left = float('inf')   # 前方偏左 0°~75°
        min_front_right = float('inf')  # 前方偏右 -75°~0°
        for i, r in enumerate(msg.ranges):
            if r <= msg.range_min or r >= msg.range_max or math.isinf(r) or math.isnan(r):
                continue
            if r > 5.0:
                continue  # 只关心 5m 以内
            angle = msg.angle_min + i * msg.angle_increment
            angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
            if 0.0 <= angle < 1.309:      # 前方偏左 0°~75°（共150°扇区）
                if r < min_front_left:
                    min_front_left = r
            if -1.309 < angle < 0.0:      # 前方偏右 -75°~0°
                if r < min_front_right:
                    min_front_right = r
        self.lidar_left_dist = min_front_left
        self.lidar_right_dist = min_front_right

    def _nav_trigger_cb(self, msg: String):
        target = msg.data.strip()
        if not target:
            return
        if self.navigating:
            self.get_logger().warn('导航进行中，忽略新请求')
            return
        self.get_logger().info(f'收到导航请求: {target}')
        self.pending_target = target

    def _mode_cb(self, msg: String):
        mode = msg.data.lower()
        if mode != 'navigation' and self.navigating:
            self.get_logger().info(f'模式切换到 {mode}，取消导航')
            self._cancel_nav()

    def _blocked_cb(self, msg: String):
        if self.navigating:
            self.blocked = True
            self.get_logger().warn(f'收到阻断信号: {msg.data}')

    # ================================================================
    #  路点图加载 + Dijkstra（复用 navigator 逻辑）
    # ================================================================

    def _load_graph(self):
        if not os.path.exists(self.data_file):
            self.get_logger().warn(f'路径图不存在: {self.data_file}')
            return
        try:
            with open(self.data_file, 'r') as f:
                data = json.load(f)
            self.waypoints = data.get('waypoints', {})
            self.edges = data.get('edges', [])
            self._build_adjacency()
            self.get_logger().info(
                f'已加载 {len(self.waypoints)} 路点, {len(self.edges)} 边')
        except Exception as e:
            self.get_logger().error(f'加载路径图失败: {e}')

    def _build_adjacency(self):
        self.adjacency = {wp_id: [] for wp_id in self.waypoints}
        for edge in self.edges:
            a, b = edge['from'], edge['to']
            d = edge.get('distance', 0.0)
            if a in self.adjacency:
                self.adjacency[a].append((b, d))
            if b in self.adjacency:
                self.adjacency[b].append((a, d))

    def _dijkstra(self, start_id, goal_id):
        if start_id == goal_id:
            return [start_id]
        heap = [(0.0, start_id)]
        dist = {start_id: 0.0}
        prev = {}
        while heap:
            cost, current = heapq.heappop(heap)
            if current == goal_id:
                path = []
                node = goal_id
                while node is not None:
                    path.append(node)
                    node = prev.get(node)
                return list(reversed(path))
            if cost > dist.get(current, float('inf')):
                continue
            for neighbor, edge_dist in self.adjacency.get(current, []):
                new_cost = cost + edge_dist
                if new_cost < dist.get(neighbor, float('inf')):
                    dist[neighbor] = new_cost
                    prev[neighbor] = current
                    heapq.heappush(heap, (new_cost, neighbor))
        return None

    def _resolve_target(self, target):
        if target in self.waypoints:
            return target
        for wp_id, wp in self.waypoints.items():
            if wp.get('label') == target:
                return wp_id
        target_lower = target.lower()
        for wp_id, wp in self.waypoints.items():
            label = wp.get('label')
            if label and label.lower() == target_lower:
                return wp_id
        return None

    def _find_nearest_wp(self, lat, lon):
        best_id, best_dist = None, float('inf')
        for wp_id, wp in self.waypoints.items():
            if 'lat' not in wp or wp['lat'] is None:
                continue
            d = haversine(lat, lon, wp['lat'], wp['lon'])
            if d < best_dist:
                best_id, best_dist = wp_id, d
        return best_id

    def _find_nearest_forward_wp(self, lat, lon):
        """找车前方 ±90° 内最近的路点"""
        if self.heading_deg is None:
            return self._find_nearest_wp(lat, lon)
        heading_rad = math.radians(self.heading_deg)
        best_id, best_dist = None, float('inf')
        for wp_id, wp in self.waypoints.items():
            if 'lat' not in wp or wp['lat'] is None:
                continue
            bearing = math.radians(compass_bearing(lat, lon, wp['lat'], wp['lon']))
            angle_diff = abs(bearing - heading_rad)
            if angle_diff > math.pi:
                angle_diff = 2.0 * math.pi - angle_diff
            if angle_diff >= math.pi / 2.0:
                continue  # 后方，跳过
            d = haversine(lat, lon, wp['lat'], wp['lon'])
            if d < best_dist:
                best_id, best_dist = wp_id, d
        return best_id

    # ================================================================
    #  路径平滑（防蛇形措施 #5: 拟合近似中心线，不穿过噪声点）
    # ================================================================

    def _smooth_path_coords(self, path_ids, iterations=3):
        """Laplacian 平滑：每个点移向邻居平均值，拟合近似平滑线。

        GPS 路点有 ±1.5m 噪声，直接追踪会蛇形。
        迭代平滑后路点漂向真实中心线，弯道自然圆滑。
        起点和终点不动，保证导航起止位置不变。
        """
        if len(path_ids) <= 2:
            return
        coords = []
        for wp_id in path_ids:
            wp = self.waypoints[wp_id]
            coords.append([wp['lat'], wp['lon']])

        for _ in range(iterations):
            new_coords = [coords[0]]
            for i in range(1, len(coords) - 1):
                lat = (coords[i - 1][0] + coords[i][0] + coords[i + 1][0]) / 3.0
                lon = (coords[i - 1][1] + coords[i][1] + coords[i + 1][1]) / 3.0
                new_coords.append([lat, lon])
            new_coords.append(coords[-1])
            coords = new_coords

        # 写回 waypoints（只改参与导航的路点，不影响原始数据文件）
        for i, wp_id in enumerate(path_ids):
            self.waypoints[wp_id]['lat'] = coords[i][0]
            self.waypoints[wp_id]['lon'] = coords[i][1]

        self.get_logger().info(
            f'路径平滑: {len(path_ids)} 路点, {iterations} 轮 Laplacian')

    # ================================================================
    #  路点稀疏化（防蛇形措施 #3）
    # ================================================================

    def _thin_waypoints(self, path_ids):
        """直线段跳过中间路点，拐点保留。减少目标切换频率，拉远目标距离。"""
        if len(path_ids) <= 2:
            return path_ids

        kept = [path_ids[0]]
        skip_count = 0

        for i in range(1, len(path_ids) - 1):
            prev_wp = self.waypoints[path_ids[i - 1]]
            curr_wp = self.waypoints[path_ids[i]]
            next_wp = self.waypoints[path_ids[i + 1]]

            turn = self._turn_angle(prev_wp, curr_wp, next_wp)

            if turn >= self.turn_thresh:
                # 拐点，必须保留
                kept.append(path_ids[i])
                skip_count = 0
            elif skip_count >= self.wp_skip:
                # 跳够了，保留一个（维持路径方向参考）
                kept.append(path_ids[i])
                skip_count = 0
            else:
                skip_count += 1

        kept.append(path_ids[-1])
        return kept

    def _turn_angle(self, wp_a, wp_b, wp_c):
        """计算 B 点的转向角度（0=直行, 90=直角弯, 180=掉头）"""
        bear_ab = compass_bearing(wp_a['lat'], wp_a['lon'],
                                  wp_b['lat'], wp_b['lon'])
        bear_bc = compass_bearing(wp_b['lat'], wp_b['lon'],
                                  wp_c['lat'], wp_c['lon'])
        diff = abs(bear_bc - bear_ab)
        if diff > 180.0:
            diff = 360.0 - diff
        return diff

    # ================================================================
    #  GPS 平滑（防蛇形措施 #1）
    # ================================================================

    def _get_smoothed_gps(self):
        """最近 N 个 GPS 读数的滚动平均，降低 ±1.5m 随机噪声"""
        if not self.gps_buffer:
            return None, None
        window = self.gps_buffer[-self.gps_window:]
        avg_lat = sum(p[0] for p in window) / len(window)
        avg_lon = sum(p[1] for p in window) / len(window)
        return avg_lat, avg_lon

    # ================================================================
    #  导航控制主循环（10Hz）
    # ================================================================

    def _nav_loop(self):
        # 处理待执行的导航请求
        if self.pending_target is not None:
            target = self.pending_target
            self.pending_target = None
            self._start_navigation(target)
            return

        if not self.navigating:
            return

        # 检查阻断（停车+通知用户，不自动后退）
        if self.blocked:
            self._stop_cmd()
            self.get_logger().error('路径阻断，停车通知用户')
            self._finish_nav('blocked')
            return

        # GPS 超时检查（超过 30 秒无 GPS 终结导航）
        now = time.monotonic()
        gps_lost_sec = now - self.gps_time
        if gps_lost_sec > self.gps_timeout:
            self._stop_cmd()
            if gps_lost_sec > 30.0:
                self._finish_nav('failed', f'GPS 丢失 {gps_lost_sec:.0f}s，终止导航')
                return
            self.get_logger().warn('GPS 超时，停车等待', throttle_duration_sec=3.0)
            return

        # 航向超时检查
        if self.heading_deg is None or now - self.heading_time > self.gps_timeout:
            self._stop_cmd()
            self.get_logger().warn('G90 航向不可用，停车等待', throttle_duration_sec=3.0)
            return

        # 获取平滑 GPS 位置
        cur_lat, cur_lon = self._get_smoothed_gps()
        if cur_lat is None:
            self._stop_cmd()
            return

        # 当前路点（用于到达/经过判定）
        target_id = self.nav_path[self.nav_index]
        target_wp = self.waypoints[target_id]
        target_lat, target_lon = target_wp['lat'], target_wp['lon']

        dist = haversine(cur_lat, cur_lon, target_lat, target_lon)
        target_bearing = compass_bearing(cur_lat, cur_lon, target_lat, target_lon)
        error_to_target = normalize_angle_deg(target_bearing - self.heading_deg)

        # 到达（距离够近）或经过（路点到身后 >90°）→ 切下一个
        passed = abs(error_to_target) > 90.0 and dist < 8.0
        if dist < self.arrival_tol or passed:
            reason = '经过' if passed else '到达'
            self.nav_index += 1
            if self.nav_index >= len(self.nav_path):
                self._stop_cmd()
                self.get_logger().info('导航完成: 已到达目标')
                self._finish_nav('arrived')
                return
            next_id = self.nav_path[self.nav_index]
            self.get_logger().info(
                f'{reason} {target_id} ({dist:.1f}m) → {next_id} '
                f'[{self.nav_index}/{len(self.nav_path)}]')
            self.pulse_state = 0
            self.pulse_ticks = 0
            return

        # === 前瞻 +1 方位角误差（保留，已验证有效） ===
        look_idx = min(self.nav_index + 1, len(self.nav_path) - 1)
        look_id = self.nav_path[look_idx]
        look_wp = self.waypoints[look_id]
        look_bearing = compass_bearing(
            cur_lat, cur_lon, look_wp['lat'], look_wp['lon'])
        error_deg = normalize_angle_deg(look_bearing - self.heading_deg)

        # === 脉冲状态机（替代连续 PID） ===
        # 优先级: 侧边护栏 > 方向修正 > 直行
        wall_left = self.lidar_left_dist < self.wall_threshold
        wall_right = self.lidar_right_dist < self.wall_threshold

        # 侧边护栏（最高优先级，可打断方向脉冲/冷却）
        if (wall_left or wall_right) and self.pulse_state not in (3, 4):
            if wall_left and wall_right:
                self.pulse_dir = 1 if self.lidar_right_dist < self.lidar_left_dist else -1
            elif wall_right:
                self.pulse_dir = 1   # 右墙 → 左推 (w>0)
            else:
                self.pulse_dir = -1  # 左墙 → 右推 (w<0)
            self.pulse_state = 3
            self.pulse_ticks = self.pulse_wall_on_ticks
        # 方向修正（仅从直行状态触发）
        elif self.pulse_state == 0 and abs(error_deg) > self.dead_zone:
            self.pulse_dir = -1 if error_deg > 0 else 1  # error>0→右转(w<0)
            self.pulse_state = 1
            self.pulse_ticks = self.pulse_nav_on_ticks

        # 状态执行 + 倒计时
        if self.pulse_state == 1:      # 方向脉冲
            angular_z = self.pulse_dir * self.pulse_nav_w
            self.pulse_ticks -= 1
            if self.pulse_ticks <= 0:
                self.pulse_state = 2
                self.pulse_ticks = self.pulse_nav_off_ticks
        elif self.pulse_state == 2:    # 方向冷却
            angular_z = 0.0
            self.pulse_ticks -= 1
            if self.pulse_ticks <= 0:
                self.pulse_state = 0
        elif self.pulse_state == 3:    # 避墙脉冲
            angular_z = self.pulse_dir * self.pulse_wall_w
            self.pulse_ticks -= 1
            if self.pulse_ticks <= 0:
                self.pulse_state = 4
                self.pulse_ticks = self.pulse_wall_off_ticks
        elif self.pulse_state == 4:    # 避墙冷却（允许方向修正打断）
            if abs(error_deg) > self.dead_zone:
                self.pulse_dir = -1 if error_deg > 0 else 1
                self.pulse_state = 1
                self.pulse_ticks = self.pulse_nav_on_ticks
                angular_z = self.pulse_dir * self.pulse_nav_w
            else:
                angular_z = 0.0
                self.pulse_ticks -= 1
                if self.pulse_ticks <= 0:
                    self.pulse_state = 0
        else:                          # 直行
            angular_z = 0.0

        # 安全上限
        angular_z = max(-self.max_angular, min(self.max_angular, angular_z))

        # 线速度（脉冲期间不减速，0.08 rad/s 很温柔）
        linear_x = min(self.max_speed, dist * self.lin_kp)
        linear_x = max(0.05, linear_x)

        # 发布
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

        _ST = ['直行', '修正', '冷却', '避墙', '墙冷']
        self.get_logger().info(
            f'近:{target_id}({dist:.0f}m) 瞄:{look_id} | '
            f'偏差:{error_deg:+.1f}° | {_ST[self.pulse_state]} '
            f'v:{linear_x:.2f} w:{angular_z:.2f} '
            f'[{self.nav_index + 1}/{len(self.nav_path)}]',
            throttle_duration_sec=1.0)

    # ================================================================
    #  导航生命周期
    # ================================================================

    def _apply_nav_session_correction(self, cur_lat, cur_lon):
        """导航启动时 session 漂移校正（多锚点平均）。

        单锚点 offset = 物理距离 + GPS漂移，物理距离污染导致路径横向偏移→蛇形。
        多锚点平均：物理距离方向各异(抵消)，漂移方向一致(保留)。
        """
        # 收集最近 N 个路点作为锚点样本
        MAX_DIST = 15.0
        samples = []
        for wp_id, wp in self.waypoints.items():
            if 'lat' not in wp or wp['lat'] is None:
                continue
            d = haversine(cur_lat, cur_lon, wp['lat'], wp['lon'])
            if d < MAX_DIST:
                samples.append((d, wp_id, wp))

        if not samples:
            self.get_logger().warn('Session校正: 15m 内无路点，跳过校正')
            return

        # 按距离排序，取最近的 N 个（最多 20 个，足够平均掉物理距离）
        samples.sort()
        use_n = min(len(samples), 20)
        samples = samples[:use_n]

        # 多锚点平均 offset
        sum_dlat = 0.0
        sum_dlon = 0.0
        for _, _, wp in samples:
            sum_dlat += cur_lat - wp['lat']
            sum_dlon += cur_lon - wp['lon']
        offset_lat = sum_dlat / use_n
        offset_lon = sum_dlon / use_n

        offset_n = offset_lat * 111320
        offset_e = offset_lon * 111320 * math.cos(math.radians(cur_lat))
        offset_m = math.sqrt(offset_n ** 2 + offset_e ** 2)

        if offset_m < 0.5:
            self.get_logger().info(
                f'Session校正: {use_n} 锚点平均偏移 {offset_m:.1f}m < 0.5m，无需校正')
            return

        # 限制最大校正 1m，防止 GPS 噪声过度校正
        if offset_m > 1.0:
            scale = 1.0 / offset_m
            offset_lat *= scale
            offset_lon *= scale
            offset_n *= scale
            offset_e *= scale
            self.get_logger().info(
                f'Session校正: 原始 {offset_m:.1f}m 超过 1m，裁剪到 1.0m')
            offset_m = 1.0

        # 平移所有路点到当前 GPS 帧
        for wp in self.waypoints.values():
            if 'lat' in wp and wp['lat'] is not None:
                wp['lat'] += offset_lat
                wp['lon'] += offset_lon

        self.get_logger().info(
            f'Session校正: {use_n} 锚点平均, '
            f'平移 {offset_m:.1f}m (北{offset_n:+.1f}m, 东{offset_e:+.1f}m), '
            f'校正 {len(self.waypoints)} 路点')

    def _start_navigation(self, target):
        """规划路径并开始脉冲式跟随"""
        self._stop_cmd()
        self.blocked = False

        self._load_graph()
        if not self.waypoints:
            self._finish_nav('failed', '无路点数据')
            return

        cur_lat, cur_lon = self._get_smoothed_gps()
        if cur_lat is None:
            self._finish_nav('failed', '无 GPS 信号')
            return

        # Session 漂移校正已移除：GPS 1.5m CEP 下校正反而引入系统偏差

        target_id = self._resolve_target(target)
        if target_id is None:
            self._finish_nav('failed', f'目标未找到: {target}')
            return

        # 找车前方最近路点作为起点（±90°内）
        start_id = self._find_nearest_forward_wp(cur_lat, cur_lon)
        if start_id is None:
            # 回退：前方没有就用最近的
            start_id = self._find_nearest_wp(cur_lat, cur_lon)
        if start_id is None:
            self._finish_nav('failed', '附近无路点')
            return

        path = self._dijkstra(start_id, target_id)
        if path is None:
            self._finish_nav('failed', f'无可达路径: {start_id} → {target_id}')
            return

        # 跳过车后方路点，从前方 ±90° 内第一个点开始
        if self.heading_deg is not None and len(path) > 1:
            heading_rad = math.radians(self.heading_deg)
            for i, wp_id in enumerate(path):
                wp = self.waypoints[wp_id]
                if 'lat' not in wp or wp['lat'] is None:
                    continue
                bearing = math.radians(compass_bearing(
                    cur_lat, cur_lon, wp['lat'], wp['lon']))
                angle_diff = abs(bearing - heading_rad)
                if angle_diff > math.pi:
                    angle_diff = 2.0 * math.pi - angle_diff
                if angle_diff < math.pi / 2.0:
                    if i > 0:
                        self.get_logger().info(
                            f'跳过车后方 {i} 个路点，从 {wp_id} 开始')
                    path = path[i:]
                    break

        self.get_logger().info(
            f'脉冲路径: {len(path)} 路点 → 逐点跟随')
        self.nav_path = path
        self.nav_index = 0
        self.navigating = True
        self.blocked = False
        self.pulse_state = 0
        self.pulse_ticks = 0
        self.pulse_dir = 0

    def _cancel_nav(self):
        if self.navigating:
            self.navigating = False
            self._stop_cmd()
            self._finish_nav('canceled')

    def _finish_nav(self, result, reason=''):
        self._stop_cmd()
        self.navigating = False
        self.nav_path = []
        self.nav_index = 0
        if reason:
            self.get_logger().error(reason) if result == 'failed' else None
        msg = String()
        msg.data = result
        self.nav_complete_pub.publish(msg)
        self.get_logger().info(f'导航结果: {result}')

    def _stop_cmd(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    # ================================================================
    #  入口
    # ================================================================


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node._stop_cmd()
        except Exception:
            pass
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
