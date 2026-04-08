"""GPS 路点导航器（v8.1: MPPI 避障模式 + PID 回退模式）

MPPI 模式 (use_mppi=true, 默认):
  /nav_trigger → Dijkstra路径 → 样条曲线平滑 → 密集path
  → FollowPath action → MPPI 避障执行 → /cmd_vel_nav
  GPS 到达判定 → 切下一段 path → 直到终点 → /nav_complete "arrived"

PID 回退模式 (use_mppi=false):
  /nav_trigger → Dijkstra → 稀疏化 → GPS-PID逐点跟随 → /cmd_vel_nav
  防蛇形: GPS平滑+死区+稀疏化+低通滤波+G90航向

接口:
  输入: /nav_trigger (String)     — 目标名、路点ID
  输出: /nav_complete (String)    — arrived / failed / blocked / canceled
  阻断: /nav_blocked (String)     — 来自 LiDAR 急停
"""

import json
import math
import os
import heapq
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

from std_msgs.msg import String, Float32
from sensor_msgs.msg import NavSatFix, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path

# MPPI 模式需要
try:
    from nav2_msgs.action import FollowPath as FollowPathAction
    from rclpy.action import ActionClient
    import tf2_ros
    _NAV2_AVAILABLE = True
except ImportError:
    _NAV2_AVAILABLE = False

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
                               os.path.expanduser('~/golf_ws/data/path_graph.json'))
        self.declare_parameter('arrival_tolerance', 5.0)     # 到达判定 (m)
        self.declare_parameter('max_speed', 0.5)             # 最大线速度 (m/s)
        self.declare_parameter('slowdown_distance', 10.0)    # 开始减速距离 (m)
        self.declare_parameter('waypoint_skip', 2)           # 直线段跳跃数
        self.declare_parameter('turn_threshold', 30.0)       # 拐点判定角度 (°)
        self.declare_parameter('bearing_dead_zone', 5.0)     # 方位角死区 (°)
        self.declare_parameter('gps_smooth_window', 3)       # GPS 平滑窗口
        self.declare_parameter('angular_smooth', 0.3)        # 角速度低通系数 (0-1, 越小越平滑)
        self.declare_parameter('ang_kp', 0.8)                # 转向 P
        self.declare_parameter('ang_kd', 0.15)               # 转向 D
        self.declare_parameter('lin_kp', 0.1)                # 速度 P
        self.declare_parameter('max_angular', 0.8)           # 最大角速度 (rad/s)
        self.declare_parameter('gps_timeout', 3.0)           # GPS 超时 (s)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('use_mppi', False)            # MPPI 模式
        self.declare_parameter('use_pure_pursuit', False)    # Pure Pursuit 模式（替代前瞻PID）
        self.declare_parameter('lookahead_distance', 10.0)   # Pure Pursuit 前瞻距离 (m)
        self.declare_parameter('spline_spacing', 2.0)        # 样条插值点间距 (m) v8.1.3: 0.5→2.0 稀疏路径不纠结GPS偏差

        self.data_file = self.get_parameter('data_file').value
        self.arrival_tol = self.get_parameter('arrival_tolerance').value
        self.max_speed = self.get_parameter('max_speed').value
        self.slowdown_dist = self.get_parameter('slowdown_distance').value
        self.wp_skip = self.get_parameter('waypoint_skip').value
        self.turn_thresh = self.get_parameter('turn_threshold').value
        self.dead_zone = self.get_parameter('bearing_dead_zone').value
        self.gps_window = self.get_parameter('gps_smooth_window').value
        self.ang_smooth = self.get_parameter('angular_smooth').value
        self.ang_kp = self.get_parameter('ang_kp').value
        self.ang_kd = self.get_parameter('ang_kd').value
        self.lin_kp = self.get_parameter('lin_kp').value
        self.max_angular = self.get_parameter('max_angular').value
        self.gps_timeout = self.get_parameter('gps_timeout').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.use_mppi = self.get_parameter('use_mppi').value and _NAV2_AVAILABLE
        self.use_pure_pursuit = self.get_parameter('use_pure_pursuit').value
        self.lookahead_dist = self.get_parameter('lookahead_distance').value
        self.spline_spacing = self.get_parameter('spline_spacing').value

        # === MPPI 模式初始化 ===
        if self.use_mppi:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.follow_path_client = ActionClient(self, FollowPathAction, 'follow_path')
            self.mppi_ready = False
            self._goal_handle = None

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

        # === PID 状态 ===
        self.prev_error_deg = 0.0   # 上一次方位角误差
        self.prev_angular_z = 0.0   # 低通滤波器状态

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
        if not self.use_mppi:
            self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.nav_complete_pub = self.create_publisher(String, '/nav_complete', 10)

        # === 定时器 ===
        self.create_timer(0.1, self._nav_loop)  # 10Hz 控制循环

        # === 加载路点图 ===
        self._load_graph()

        if self.use_mppi:
            mode_label = 'MPPI样条避障'
        elif self.use_pure_pursuit:
            mode_label = f'Pure Pursuit (L={self.lookahead_dist}m)'
        else:
            mode_label = f'前瞻PID (死区={self.dead_zone}°)'
        self.get_logger().info(
            f'GPS 路点导航器启动 | 模式: {mode_label} | 路点: {len(self.waypoints)} | '
            f'到达容差: {self.arrival_tol}m | kp={self.ang_kp} max_w={self.max_angular}')

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
        """从 /scan 提取前方左/前方右最近距离（前方 3m 扇区）"""
        min_front_left = float('inf')   # 前方偏左 0°~60°
        min_front_right = float('inf')  # 前方偏右 0°~-60°
        for i, r in enumerate(msg.ranges):
            if r <= msg.range_min or r >= msg.range_max or math.isinf(r) or math.isnan(r):
                continue
            if r > 3.0:
                continue  # 只关心 3m 以内
            angle = msg.angle_min + i * msg.angle_increment
            angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
            if 0.0 <= angle < 1.05:       # 前方偏左 0°~60°
                if r < min_front_left:
                    min_front_left = r
            if -1.05 < angle < 0.0:       # 前方偏右 -60°~0°
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

    def _get_avg_heading(self):
        """最近航向读数的圆周平均（处理 359°/1° 跨越问题），减少初始化噪声"""
        if not self.heading_buffer:
            return self.heading_deg
        # 只用最近 10 个读数 (~1秒)
        recent = self.heading_buffer[-10:]
        # 圆周平均：先转极坐标再平均
        sin_sum = sum(math.sin(math.radians(h)) for h, _ in recent)
        cos_sum = sum(math.cos(math.radians(h)) for h, _ in recent)
        avg = math.degrees(math.atan2(sin_sum, cos_sum)) % 360.0
        return avg

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

        # MPPI 模式: GPS 到达判定 + 切段（MPPI 负责执行，不做 PID）
        if self.use_mppi:
            self._mppi_nav_loop()
            return

        # === 以下为 PID 回退模式 ===

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
            self.prev_error_deg = 0.0
            return

        # === 转向计算：Pure Pursuit 或 前瞻PID ===
        if self.use_pure_pursuit:
            error_deg, look_label = self._pure_pursuit_error(
                cur_lat, cur_lon)
        else:
            error_deg, look_label = self._lookahead_pid_error(
                cur_lat, cur_lon, dist)

        # P 控制
        error_rad = math.radians(error_deg)
        raw_angular = -(self.ang_kp * error_rad)

        # 低通滤波
        angular_z = (self.ang_smooth * raw_angular
                     + (1.0 - self.ang_smooth) * self.prev_angular_z)
        self.prev_angular_z = angular_z

        # LiDAR 侧边护栏：微推避障
        WALL_NUDGE = 0.05
        if self.lidar_right_dist < 2.0 and self.lidar_left_dist < 2.0:
            if self.lidar_right_dist < self.lidar_left_dist:
                angular_z += WALL_NUDGE
            else:
                angular_z -= WALL_NUDGE
        elif self.lidar_right_dist < 2.0:
            angular_z += WALL_NUDGE
        elif self.lidar_left_dist < 2.0:
            angular_z -= WALL_NUDGE

        # 硬上限
        angular_z = max(-self.max_angular, min(self.max_angular, angular_z))

        # 线速度
        linear_x = min(self.max_speed, dist * self.lin_kp)
        turn_factor = 1.0 - min(0.5, abs(angular_z) / self.max_angular)
        linear_x *= turn_factor
        linear_x = max(0.05, linear_x)

        # 发布
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'近:{target_id}({dist:.0f}m) {look_label} | '
            f'偏差:{error_deg:+.1f}° | v:{linear_x:.2f} w:{angular_z:.2f} '
            f'[{self.nav_index + 1}/{len(self.nav_path)}]',
            throttle_duration_sec=1.0)

    # ================================================================
    #  转向策略
    # ================================================================

    def _lookahead_pid_error(self, cur_lat, cur_lon, dist_to_target):
        """前瞻PID：看远3个路点的方位角误差 + 终点路段方向修复"""
        look_idx = min(self.nav_index + 3, len(self.nav_path) - 1)
        look_id = self.nav_path[look_idx]
        look_wp = self.waypoints[look_id]

        # 终点修复：前瞻钳位到末尾且距离近时，用路段方向代替GPS方位角
        is_terminal = (look_idx == len(self.nav_path) - 1)
        look_dist = haversine(cur_lat, cur_lon,
                              look_wp['lat'], look_wp['lon'])
        if is_terminal and look_dist < 2.0 * self.arrival_tol and look_idx > 0:
            # 用前一路点→终点的路段方向（固定值，不受GPS噪声影响）
            prev_id = self.nav_path[look_idx - 1]
            prev_wp = self.waypoints[prev_id]
            seg_bearing = compass_bearing(
                prev_wp['lat'], prev_wp['lon'],
                look_wp['lat'], look_wp['lon'])
            error_deg = normalize_angle_deg(seg_bearing - self.heading_deg)
            label = f'终:{look_id}(段{seg_bearing:.0f}°)'
        else:
            look_bearing = compass_bearing(
                cur_lat, cur_lon, look_wp['lat'], look_wp['lon'])
            error_deg = normalize_angle_deg(look_bearing - self.heading_deg)
            label = f'瞄:{look_id}'

        # 死区
        if abs(error_deg) < self.dead_zone:
            error_deg = 0.0

        return error_deg, label

    def _pure_pursuit_error(self, cur_lat, cur_lon):
        """Pure Pursuit：路线上固定距离前瞻点的方位角误差"""
        # 1. 找最近路段
        seg_idx = self._find_nearest_segment(cur_lat, cur_lon)

        # 2. 沿路线前移 L 米找追踪点
        look_lat, look_lon = self._walk_along_path(
            seg_idx, cur_lat, cur_lon, self.lookahead_dist)

        # 3. 算方位角误差（不需要死区，前瞻距离固定，噪声恒定）
        look_bearing = compass_bearing(cur_lat, cur_lon, look_lat, look_lon)
        error_deg = normalize_angle_deg(look_bearing - self.heading_deg)

        label = f'PP:seg{seg_idx}(L={self.lookahead_dist:.0f}m)'
        return error_deg, label

    def _find_nearest_segment(self, lat, lon):
        """找车最近的路段（投影距离最短）"""
        best_seg = max(0, self.nav_index - 1)
        best_dist = float('inf')
        # 只搜索 nav_index 附近的段，避免跳到远处
        start = max(0, self.nav_index - 2)
        end = min(len(self.nav_path) - 1, self.nav_index + 5)
        for i in range(start, end):
            wp_a = self.waypoints[self.nav_path[i]]
            wp_b = self.waypoints[self.nav_path[i + 1]]
            d = self._point_to_segment_dist(lat, lon, wp_a, wp_b)
            if d < best_dist:
                best_dist = d
                best_seg = i
        return best_seg

    def _point_to_segment_dist(self, lat, lon, wp_a, wp_b):
        """点到线段的垂直距离（米）"""
        # 转局部米制坐标
        cos_lat = math.cos(math.radians(lat))
        dp_n = (lat - wp_a['lat']) * 111320.0
        dp_e = (lon - wp_a['lon']) * 111320.0 * cos_lat
        db_n = (wp_b['lat'] - wp_a['lat']) * 111320.0
        db_e = (wp_b['lon'] - wp_a['lon']) * 111320.0 * cos_lat
        seg_len_sq = db_n * db_n + db_e * db_e
        if seg_len_sq < 0.01:
            return math.sqrt(dp_n * dp_n + dp_e * dp_e)
        t = max(0.0, min(1.0, (dp_n * db_n + dp_e * db_e) / seg_len_sq))
        proj_n = t * db_n
        proj_e = t * db_e
        return math.sqrt((dp_n - proj_n) ** 2 + (dp_e - proj_e) ** 2)

    def _walk_along_path(self, seg_idx, cur_lat, cur_lon, L):
        """从最近路段的投影点出发，沿路线前行 L 米，返回追踪点坐标"""
        # 投影到最近路段，算出剩余段内距离
        wp_a = self.waypoints[self.nav_path[seg_idx]]
        wp_b = self.waypoints[self.nav_path[seg_idx + 1]]
        cos_lat = math.cos(math.radians(cur_lat))
        dp_n = (cur_lat - wp_a['lat']) * 111320.0
        dp_e = (cur_lon - wp_a['lon']) * 111320.0 * cos_lat
        db_n = (wp_b['lat'] - wp_a['lat']) * 111320.0
        db_e = (wp_b['lon'] - wp_a['lon']) * 111320.0 * cos_lat
        seg_len_sq = db_n * db_n + db_e * db_e
        seg_len = math.sqrt(seg_len_sq) if seg_len_sq > 0.01 else 0.0

        if seg_len > 0:
            t = max(0.0, min(1.0,
                             (dp_n * db_n + dp_e * db_e) / seg_len_sq))
            remaining_in_seg = seg_len * (1.0 - t)
        else:
            remaining_in_seg = 0.0

        remaining = L

        # 先消耗当前段剩余部分
        if remaining <= remaining_in_seg and seg_len > 0:
            frac = t + remaining / seg_len
            lat = wp_a['lat'] + frac * (wp_b['lat'] - wp_a['lat'])
            lon = wp_a['lon'] + frac * (wp_b['lon'] - wp_a['lon'])
            return lat, lon
        remaining -= remaining_in_seg

        # 继续沿后续路段前行
        for i in range(seg_idx + 1, len(self.nav_path) - 1):
            wa = self.waypoints[self.nav_path[i]]
            wb = self.waypoints[self.nav_path[i + 1]]
            d = haversine(wa['lat'], wa['lon'], wb['lat'], wb['lon'])
            if remaining <= d and d > 0:
                frac = remaining / d
                lat = wa['lat'] + frac * (wb['lat'] - wa['lat'])
                lon = wa['lon'] + frac * (wb['lon'] - wa['lon'])
                return lat, lon
            remaining -= d

        # 路线走完了，返回终点
        final = self.waypoints[self.nav_path[-1]]
        return final['lat'], final['lon']

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
        """规划路径并开始 GPS-PID 跟随"""
        # 清理上一次残留的 goal
        self._stop_cmd()
        self._goal_handle = None
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

        if self.use_mppi:
            self.nav_path = path
            self.nav_index = 0
            self.navigating = True
            self.blocked = False
            self._send_spline_path(path, cur_lat, cur_lon)
        else:
            self.get_logger().info(
                f'PID路径: {len(path)} 路点 → 逐点跟随')
            self.nav_path = path
            self.nav_index = 0
            self.navigating = True
            self.blocked = False
            self.prev_error_deg = 0.0
            self.prev_angular_z = 0.0

    def _cancel_nav(self):
        if self.navigating:
            self.navigating = False
            self._stop_cmd()
            self._finish_nav('canceled')

    def _finish_nav(self, result, reason=''):
        self._stop_cmd()  # 确保取消残留的 FollowPath goal
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
        if self.use_mppi:
            # MPPI 模式: 取消 FollowPath goal，controller_server 会自动停车
            if self._goal_handle is not None:
                try:
                    self._goal_handle.cancel_goal_async()
                except Exception:
                    pass
                self._goal_handle = None
        else:
            cmd = Twist()
            self.cmd_pub.publish(cmd)

    # ================================================================
    #  MPPI 模式: 样条平滑 + FollowPath + GPS 到达判定
    # ================================================================

    def _mppi_nav_loop(self):
        """MPPI 模式主循环: 仅做 GPS 到达判定，MPPI 负责执行"""
        # 阻断检查
        if self.blocked:
            self.get_logger().error('路径阻断，停车通知用户')
            self._finish_nav('blocked')
            return

        # GPS 到达判定
        cur_lat, cur_lon = self._get_smoothed_gps()
        if cur_lat is None:
            return

        # 检查是否到达终点
        final_id = self.nav_path[-1]
        final_wp = self.waypoints[final_id]
        dist_to_final = haversine(cur_lat, cur_lon, final_wp['lat'], final_wp['lon'])

        if dist_to_final < self.arrival_tol:
            self.get_logger().info(f'到达终点 {final_id} ({dist_to_final:.1f}m)')
            self._finish_nav('arrived')
            return

        self.get_logger().info(
            f'MPPI 导航中 | 距终点: {dist_to_final:.1f}m | '
            f'终点: {final_id} [{len(self.nav_path)} 路点]',
            throttle_duration_sec=3.0)

    def _send_spline_path(self, path_ids, ref_lat, ref_lon):
        """GPS 路点 → 旋转对齐 odom → 样条平滑 → FollowPath action

        关键：GPS local 坐标 (x=East, y=North) 必须旋转到 odom 帧后才能用。
        odom 帧的 x 轴方向取决于 EKF 初始化，不一定朝东。
        用 G90 航向 (compass) + odom yaw 计算旋转角。
        """
        if not self.mppi_ready:
            if not self.follow_path_client.server_is_ready():
                self.get_logger().warn('MPPI controller_server 未就绪')
                self._finish_nav('failed', 'MPPI controller_server 未就绪，请先启动 minimal_nav2.launch.py')
                return
            self.mppi_ready = True
            self.get_logger().info('MPPI controller_server 就绪')

        # 需要 G90 航向来计算 GPS→odom 旋转（用平均值减少初始化噪声）
        if self.heading_deg is None:
            self._finish_nav('failed', 'G90 航向不可用，无法对齐 GPS→odom')
            return

        avg_heading = self._get_avg_heading()

        # 获取 robot 在 odom 帧的位姿（位置+朝向）
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom_combined', 'base_footprint', rclpy.time.Time(),
                timeout=Duration(seconds=1.0))
            anchor_x = tf.transform.translation.x
            anchor_y = tf.transform.translation.y
            # 从 quaternion 提取 yaw
            q = tf.transform.rotation
            odom_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        except Exception:
            self._finish_nav('failed', '无法获取 TF odom→base 位姿')
            return

        # 计算 East-North → odom 旋转角
        # GPS heading: compass degrees (0=N, 90=E, clockwise)
        # → ENU heading: rad from East, CCW = pi/2 - heading_rad
        # odom yaw: rad from odom-x, CCW
        # 两者描述同一个物理方向（车头朝向），所以:
        #   heading_enu = odom_rotation + odom_yaw
        #   odom_rotation = heading_enu - odom_yaw
        heading_enu = math.pi / 2.0 - math.radians(avg_heading)
        odom_rotation = heading_enu - odom_yaw  # East-North 帧相对 odom 帧的旋转

        cos_r = math.cos(-odom_rotation)  # 负号：从 EN 旋转到 odom
        sin_r = math.sin(-odom_rotation)

        self.get_logger().info(
            f'GPS→odom 对齐: heading_avg={avg_heading:.1f}° (latest={self.heading_deg:.1f}°) '
            f'odom_yaw={math.degrees(odom_yaw):.1f}° '
            f'rotation={math.degrees(-odom_rotation):.1f}°')

        # GPS → local (x=East, y=North)，以当前 GPS 为原点
        en_points = [(0.0, 0.0)]  # 车当前位置
        for wp_id in path_ids:
            wp = self.waypoints[wp_id]
            x, y = self._gps_to_local(wp['lat'], wp['lon'], ref_lat, ref_lon)
            en_points.append((x, y))

        if len(en_points) < 2:
            self._finish_nav('failed', '路径点不足')
            return

        # 旋转 East-North → odom 帧
        local_points = []
        for ex, ny in en_points:
            ox = ex * cos_r - ny * sin_r
            oy = ex * sin_r + ny * cos_r
            local_points.append((ox, oy))

        # 样条插值
        smooth_points = self._catmull_rom_spline(local_points, self.spline_spacing)

        # 构建 Nav2 Path（odom_combined 帧），每个 pose 带沿路径方向的 orientation
        nav_path = Path()
        nav_path.header.frame_id = 'odom_combined'
        nav_path.header.stamp = self.get_clock().now().to_msg()

        for i, (lx, ly) in enumerate(smooth_points):
            pose = PoseStamped()
            pose.header = nav_path.header
            pose.pose.position.x = anchor_x + lx
            pose.pose.position.y = anchor_y + ly
            if i == 0:
                # 第一个点是车当前位置，yaw 必须用车实际朝向（odom_yaw）
                yaw = odom_yaw
            elif i < len(smooth_points) - 1:
                dx = smooth_points[i + 1][0] - lx
                dy = smooth_points[i + 1][1] - ly
                yaw = math.atan2(dy, dx)
            else:
                dx = lx - smooth_points[i - 1][0]
                dy = ly - smooth_points[i - 1][1]
                yaw = math.atan2(dy, dx)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            nav_path.poses.append(pose)

        # 发送 FollowPath
        goal = FollowPathAction.Goal()
        goal.path = nav_path
        future = self.follow_path_client.send_goal_async(goal)
        future.add_done_callback(self._mppi_goal_response_cb)

        self.get_logger().info(
            f'MPPI 路径已发送: {len(path_ids)} GPS路点 → '
            f'{len(smooth_points)} 样条点 (间距 {self.spline_spacing}m)')

    def _mppi_goal_response_cb(self, future):
        try:
            self._goal_handle = future.result()
            if self._goal_handle and self._goal_handle.accepted:
                # 注册 result callback 检测 abort/failure
                self._goal_handle.get_result_async().add_done_callback(
                    self._mppi_result_cb)
            else:
                self.get_logger().error('FollowPath goal 被拒绝')
                self._finish_nav('failed', 'MPPI 拒绝路径')
        except Exception as e:
            self.get_logger().error(f'FollowPath goal 发送失败: {e}')
            self._goal_handle = None
            self._finish_nav('failed', 'FollowPath goal 发送异常')

    def _mppi_result_cb(self, future):
        """FollowPath 执行结束回调（成功/abort/失败）"""
        try:
            result = future.result()
            status = result.status
            # status: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
            if status == 4:
                self.get_logger().info('FollowPath 执行完成')
            elif status == 6:
                self.get_logger().warn('FollowPath 被 abort（MPPI 无法推进）')
                if self.navigating:
                    self._finish_nav('failed', 'MPPI 执行失败，无法推进')
            elif status == 5:
                self.get_logger().info('FollowPath 已取消')
            else:
                self.get_logger().warn(f'FollowPath 未知状态: {status}')
                if self.navigating:
                    self._finish_nav('failed', f'MPPI 异常状态 {status}')
        except Exception as e:
            self.get_logger().error(f'FollowPath result 回调异常: {e}')
            if self.navigating:
                self._finish_nav('failed', 'FollowPath result 获取失败')

    def _gps_to_local(self, lat, lon, ref_lat, ref_lon):
        """GPS → 以 ref 为原点的 local (x=东, y=北) 坐标 (米)"""
        x = haversine(ref_lat, ref_lon, ref_lat, lon)
        if lon < ref_lon:
            x = -x
        y = haversine(ref_lat, ref_lon, lat, ref_lon)
        if lat < ref_lat:
            y = -y
        return x, y

    def _catmull_rom_spline(self, points, spacing):
        """Catmull-Rom 样条插值，生成密集平滑路径点。
        不依赖 scipy，纯 numpy-free 手写实现。

        输入: [(x0,y0), (x1,y1), ...] 离散控制点
        输出: [(x,y), ...] 密集平滑路径点，间距 ≈ spacing 米
        """
        if len(points) <= 1:
            return list(points)

        # Catmull-Rom 需要前后各延伸一个虚拟点
        pts = list(points)
        # 前延伸: 2*P0 - P1
        pts.insert(0, (2 * pts[0][0] - pts[1][0], 2 * pts[0][1] - pts[1][1]))
        # 后延伸: 2*Pn - Pn-1
        pts.append((2 * pts[-1][0] - pts[-2][0], 2 * pts[-1][1] - pts[-2][1]))

        result = []
        for i in range(1, len(pts) - 2):
            p0 = pts[i - 1]
            p1 = pts[i]
            p2 = pts[i + 1]
            p3 = pts[i + 2]

            # 段长度估算
            seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            n_steps = max(2, int(seg_len / spacing))

            for step in range(n_steps):
                t = step / n_steps
                t2 = t * t
                t3 = t2 * t

                # Catmull-Rom 矩阵公式
                x = 0.5 * ((2 * p1[0])
                           + (-p0[0] + p2[0]) * t
                           + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2
                           + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3)
                y = 0.5 * ((2 * p1[1])
                           + (-p0[1] + p2[1]) * t
                           + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2
                           + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3)
                result.append((x, y))

        # 添加终点
        result.append(points[-1])
        return result

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
            if not node.use_mppi:
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
