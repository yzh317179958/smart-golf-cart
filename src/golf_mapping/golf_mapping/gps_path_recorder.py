"""P1-16 路点记录器（v3.1 跨 Session 帧对齐）

数据源模式（参数 coordinate_source）：
  - raw（默认）: /gps/fix 裸 GPS lat/lon，多次经过加权平均提升精度
  - ekf（备用）: /gps/filtered EKF 平滑坐标（v2.6.0 行为，可切回）

架构：
  - 距离判断: TF odom_combined→base_footprint（EKF#1 里程计平滑，3m均匀间隔）
  - 路点坐标: 由 coordinate_source 参数决定（raw=/gps/fix, ekf=/gps/filtered）
  - 速度过滤: /odom_combined speed > 0.05（防静态 GPS 漂移记录）
  - 合并判断: GPS haversine（跨会话全局一致，无 odom 漂移问题）
  - 合并时坐标加权平均: new = (old * count + current) / (count + 1)

跨 Session 帧对齐（v3.1）：
  GPS 系统性漂移 3-8m 导致不同 session 路点在不同 GPS 帧中。
  当新 session 首次经过已有路点区域时，检测帧差（offset），
  回溯校正本 session 已记录的路点，使其对齐到已有路点图的帧。
  原理类似 SLAM 回环校正：连接点处消除累积漂移。
"""

import json
import math
import os
import re
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import tf2_ros

# 地球半径（米）
_EARTH_R = 6371000.0


def haversine(lat1, lon1, lat2, lon2):
    """两个 GPS 坐标间的地球表面距离（米）"""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2.0) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(dlon / 2.0) ** 2)
    return _EARTH_R * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


class GpsPathRecorder(Node):

    def __init__(self):
        super().__init__('gps_path_recorder')

        # 参数
        self.declare_parameter('min_record_distance', 3.0)
        self.declare_parameter('merge_distance', 3.0)  # v3.2: 5→3 缩小融合半径
        self.declare_parameter('save_interval', 300.0)
        self.declare_parameter('min_speed', 0.05)
        self.declare_parameter('data_file',
                               os.path.expanduser('~/golf_ws/data/path_graph.json'))
        self.declare_parameter('coordinate_source', 'raw')  # 'raw' | 'ekf'
        self.declare_parameter('align_distance', 8.0)  # 帧对齐检测距离（需 > 典型 GPS 漂移 5-8m）

        self.min_record_dist = self.get_parameter('min_record_distance').value
        self.merge_dist = self.get_parameter('merge_distance').value
        self.save_interval = self.get_parameter('save_interval').value
        self.min_speed = self.get_parameter('min_speed').value
        self.data_file = self.get_parameter('data_file').value
        self.coord_source = self.get_parameter('coordinate_source').value
        if self.coord_source not in ('raw', 'ekf'):
            self.get_logger().warn(
                f'coordinate_source={self.coord_source} 无效，回退到 raw')
            self.coord_source = 'raw'
        self.align_dist = self.get_parameter('align_distance').value

        # 状态
        self.follow_state = 'idle'
        self.prev_follow_state = 'idle'
        self.last_record_wp = None
        self.last_record_odom = None  # (x, y) odom位置（用于顺序距离判断）
        self.pending_label = None
        self.dirty = False
        self.wp_counter = 0
        self.current_speed = 0.0

        # 跨 Session 帧对齐状态（多锚点累积平均，消除物理距离污染）
        self.session_offset_lat = 0.0  # 当前 session 的 GPS 帧偏移（累积平均）
        self.session_offset_lon = 0.0
        self.session_aligned = False    # 是否已完成首次帧对齐
        self.session_wp_ids = []        # 本 session 新增的路点 ID（用于回溯校正）
        self._align_samples = []        # [(offset_lat, offset_lon), ...] 多锚点样本
        self._align_seen_anchors = set()  # 已采样的锚点ID（每个锚点只采一次）

        # GPS 数据
        self.last_gps_fix = None       # /gps/fix 原始（备用）
        self.last_gps_filtered = None  # /gps/filtered EKF平滑（首选）

        # 路径图数据
        self.waypoints = {}
        self.edges = []

        # TF（用于 odom_combined→base_link 距离判断）
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 加载已有数据
        self._load()

        # 订阅
        gps_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(NavSatFix, '/gps/fix', self._gps_fix_cb, gps_qos)
        self.create_subscription(NavSatFix, '/gps/filtered', self._gps_filtered_cb, gps_qos)
        self.create_subscription(String, '/follow_state', self._state_cb, 10)
        self.create_subscription(String, '/mark_waypoint_label', self._mark_cb, 10)
        self.create_subscription(Odometry, '/odom_combined', self._odom_cb, 10)

        # 发布
        self.stats_pub = self.create_publisher(String, '/path_graph/stats', 10)

        # 定时器
        self.create_timer(0.1, self._tf_timer_cb)       # 10Hz TF 轮询
        self.create_timer(5.0, self._stats_timer_cb)
        self.create_timer(self.save_interval, self._auto_save_cb)

        src_label = '裸GPS /gps/fix' if self.coord_source == 'raw' else 'EKF /gps/filtered'
        self.get_logger().info(
            f'路点记录器启动（v3.0）| 数据源: {src_label} | '
            f'已有路点: {len(self.waypoints)} | '
            f'记录间隔: {self.min_record_dist}m | 文件: {self.data_file}')

    # === GPS 坐标获取 ===

    def _get_gps_coords(self):
        """获取 GPS 坐标，由 coordinate_source 参数决定数据源。
        raw 模式: /gps/fix 裸 GPS（默认，v8.0 GPS-PID 架构）
        ekf 模式: 优先 /gps/filtered EKF 平滑，回退 /gps/fix（v2.6.0 备用）
        """
        if self.coord_source == 'ekf':
            if self.last_gps_filtered is not None:
                return self.last_gps_filtered.latitude, self.last_gps_filtered.longitude
            if self.last_gps_fix is not None:
                return self.last_gps_fix.latitude, self.last_gps_fix.longitude
        else:  # raw
            if self.last_gps_fix is not None:
                return self.last_gps_fix.latitude, self.last_gps_fix.longitude
        return None, None

    # === 回调 ===

    def _gps_fix_cb(self, msg: NavSatFix):
        """缓存 /gps/fix 原始 GPS（备用）"""
        if msg.status.status < 0 or (msg.latitude == 0.0 and msg.longitude == 0.0):
            return
        self.last_gps_fix = msg

    def _gps_filtered_cb(self, msg: NavSatFix):
        """缓存 /gps/filtered EKF平滑 GPS（首选）"""
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return
        self.last_gps_filtered = msg

    def _odom_cb(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.sqrt(vx * vx + vy * vy)

    def _state_cb(self, msg: String):
        self.prev_follow_state = self.follow_state
        self.follow_state = msg.data
        if msg.data == 'tracking' and self.prev_follow_state != 'tracking':
            self._reconnect_on_resume()

    def _reconnect_on_resume(self):
        """跟随恢复时，用 GPS haversine 连接到最近已有路点"""
        lat, lon = self._get_gps_coords()
        if lat is None or not self.waypoints:
            return
        nearest_id, nearest_dist = self._find_nearest_gps(lat, lon)
        if nearest_id is not None and nearest_dist < 20.0:
            self.last_record_wp = nearest_id
            # 同步 odom 位置
            pos = self._get_tf_position()
            if pos is not None:
                self.last_record_odom = (pos[0], pos[1])
            self.get_logger().info(
                f'跟随恢复，重连到路点 {nearest_id} (GPS距离 {nearest_dist:.1f}m)')

    def _mark_cb(self, msg: String):
        self.pending_label = msg.data
        self.get_logger().info(f'准备标记路点: {msg.data}')
        lat, lon = self._get_gps_coords()
        if lat is None:
            self.get_logger().warn('无 GPS 信号，暂存标签等待就绪')
            return
        pos = self._get_tf_position()
        if pos is not None:
            self._record_waypoint(pos[0], pos[1], force_label=msg.data)

    def _get_tf_position(self):
        """从 TF 读取 odom_combined→base_footprint（EKF#1 里程计，用于距离判断）"""
        try:
            t = self.tf_buffer.lookup_transform(
                'odom_combined', 'base_footprint', rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def _tf_timer_cb(self):
        """10Hz TF 轮询，记录路点"""
        if self.follow_state != 'tracking':
            return
        if self.current_speed < self.min_speed:
            return
        lat, lon = self._get_gps_coords()
        if lat is None:
            return

        pos = self._get_tf_position()
        if pos is None:
            return

        self._record_waypoint(pos[0], pos[1])

    # === 跨 Session 帧对齐 ===

    def _try_session_align(self, raw_lat, raw_lon):
        """多锚点累积平均帧对齐。

        单个锚点的 offset = 物理距离 + GPS漂移，物理距离污染漂移估算。
        多个锚点采样后平均：物理距离方向各异(抵消)，漂移方向一致(保留)。
        每 10Hz 调用，持续收集新锚点样本，offset 逐渐收敛到真实漂移。
        """
        if not self.waypoints:
            return

        # 在已有路点（非本 session）中找最近的
        best_id = None
        best_dist = float('inf')
        for wp_id, wp in self.waypoints.items():
            if wp_id in self.session_wp_ids:
                continue
            if 'lat' not in wp or wp['lat'] is None:
                continue
            d = haversine(raw_lat, raw_lon, wp['lat'], wp['lon'])
            if d < best_dist:
                best_dist = d
                best_id = wp_id

        if best_id is None or best_dist > self.align_dist:
            return

        # 每个锚点只采样一次（避免反复经过同一点导致权重偏斜）
        if best_id in self._align_seen_anchors:
            return
        self._align_seen_anchors.add(best_id)

        # 收集新样本
        anchor = self.waypoints[best_id]
        sample_lat = anchor['lat'] - raw_lat
        sample_lon = anchor['lon'] - raw_lon
        self._align_samples.append((sample_lat, sample_lon))

        # 计算累积平均 offset
        n = len(self._align_samples)
        new_offset_lat = sum(s[0] for s in self._align_samples) / n
        new_offset_lon = sum(s[1] for s in self._align_samples) / n

        offset_n = new_offset_lat * 111320
        offset_e = new_offset_lon * 111320 * math.cos(math.radians(raw_lat))
        offset_m = math.sqrt(offset_n ** 2 + offset_e ** 2)

        if not self.session_aligned:
            # 首次对齐：设置 offset 并回溯校正已有 session 路点
            self.session_offset_lat = new_offset_lat
            self.session_offset_lon = new_offset_lon
            self.session_aligned = True

            if self.session_wp_ids:
                for wp_id in self.session_wp_ids:
                    if wp_id in self.waypoints:
                        self.waypoints[wp_id]['lat'] += new_offset_lat
                        self.waypoints[wp_id]['lon'] += new_offset_lon
                self.get_logger().info(
                    f'首次帧对齐: 回溯校正 {len(self.session_wp_ids)} 个路点')
                self.dirty = True

            self.get_logger().info(
                f'帧对齐[{n}]: 锚点 {best_id} ({best_dist:.1f}m), '
                f'offset=({offset_n:+.1f}m北, {offset_e:+.1f}m东) = {offset_m:.1f}m')
        else:
            # 后续样本：计算增量差，修正所有 session 路点
            delta_lat = new_offset_lat - self.session_offset_lat
            delta_lon = new_offset_lon - self.session_offset_lon
            delta_m = math.sqrt((delta_lat * 111320) ** 2 +
                                (delta_lon * 111320 * math.cos(math.radians(raw_lat))) ** 2)

            if delta_m > 0.1:  # 增量 > 0.1m 才更新（避免微调抖动）
                for wp_id in self.session_wp_ids:
                    if wp_id in self.waypoints:
                        self.waypoints[wp_id]['lat'] += delta_lat
                        self.waypoints[wp_id]['lon'] += delta_lon
                self.dirty = True

            self.session_offset_lat = new_offset_lat
            self.session_offset_lon = new_offset_lon

            self.get_logger().info(
                f'帧对齐[{n}]: +锚点 {best_id} ({best_dist:.1f}m), '
                f'avg_offset=({offset_n:+.1f}m北, {offset_e:+.1f}m东) = {offset_m:.1f}m, '
                f'增量 {delta_m:.2f}m')

    def _apply_session_offset(self, raw_lat, raw_lon):
        """对当前 GPS 坐标应用 session 帧偏移"""
        return (raw_lat + self.session_offset_lat,
                raw_lon + self.session_offset_lon)

    # === 核心逻辑 ===

    def _record_waypoint(self, odom_x, odom_y, force_label=None):
        now_str = time.strftime('%Y-%m-%dT%H:%M:%S')

        # 距上一个记录点是否够远（odom 欧式距离，里程计平滑）
        if self.last_record_odom is not None and not force_label:
            dist_to_last = math.hypot(
                odom_x - self.last_record_odom[0],
                odom_y - self.last_record_odom[1])
            if dist_to_last < self.min_record_dist:
                return

        # 获取 GPS 坐标
        raw_lat, raw_lon = self._get_gps_coords()
        if raw_lat is None:
            return

        # 跨 Session 帧对齐：检测是否进入已有路点区域
        self._try_session_align(raw_lat, raw_lon)

        # 应用帧偏移（未对齐时 offset=0，无影响）
        gps_lat, gps_lon = self._apply_session_offset(raw_lat, raw_lon)

        # 检查是否靠近已有路点（GPS haversine，用校正后坐标）
        nearest_id, nearest_dist = self._find_nearest_gps(gps_lat, gps_lon)

        if nearest_id is not None and nearest_dist < self.merge_dist:
            wp = self.waypoints[nearest_id]
            n = wp['traverse_count']
            # GPS 坐标加权平均（用校正后坐标参与平均）
            wp['lat'] = (wp['lat'] * n + gps_lat) / (n + 1)
            wp['lon'] = (wp['lon'] * n + gps_lon) / (n + 1)
            wp['traverse_count'] = n + 1
            wp['last_traversed'] = now_str
            if wp['blocked_count'] > 0:
                wp['blocked_count'] = 0
            if force_label and not wp.get('label'):
                wp['label'] = force_label
            self.pending_label = None
            self._add_edge(self.last_record_wp, nearest_id)
            self.last_record_wp = nearest_id
            self.last_record_odom = (odom_x, odom_y)
            self.dirty = True
            self._save()
            return

        # 创建新路点（用校正后坐标）
        label = force_label or self.pending_label
        self.pending_label = None

        if label:
            safe_label = re.sub(r'[^a-zA-Z0-9_\u4e00-\u9fff]', '_', label)
            base_id = safe_label.lower()[:32]
            wp_id = base_id
            while wp_id in self.waypoints:
                self.wp_counter += 1
                wp_id = f'{base_id}_{self.wp_counter}'
        else:
            self.wp_counter += 1
            wp_id = f'wp_{self.wp_counter:04d}'

        self.waypoints[wp_id] = {
            'lat': gps_lat,
            'lon': gps_lon,
            'last_traversed': now_str,
            'traverse_count': 1,
            'blocked_count': 0,
            'label': label,
        }

        self._add_edge(self.last_record_wp, wp_id)
        self.last_record_wp = wp_id
        self.last_record_odom = (odom_x, odom_y)
        self.session_wp_ids.append(wp_id)  # 记录本 session 新增路点
        self.dirty = True
        self._save()

        aligned_tag = '已对齐' if self.session_aligned else '未对齐'
        src = self.coord_source
        log_label = f' [{label}]' if label else ''
        self.get_logger().info(
            f'新路点 {wp_id}{log_label} GPS({gps_lat:.6f},{gps_lon:.6f})({src},{aligned_tag}) '
            f'odom({odom_x:.1f},{odom_y:.1f}) | 总计: {len(self.waypoints)}')

    def _find_nearest_gps(self, lat, lon):
        """用 GPS haversine 找最近路点（跨会话全局一致）"""
        nearest_id = None
        nearest_dist = float('inf')
        for wp_id, wp in self.waypoints.items():
            if 'lat' not in wp or wp['lat'] is None:
                continue
            d = haversine(lat, lon, wp['lat'], wp['lon'])
            if d < nearest_dist:
                nearest_dist = d
                nearest_id = wp_id
        if nearest_id is None:
            return None, None
        return nearest_id, nearest_dist

    def _add_edge(self, from_id, to_id):
        if from_id is None or to_id is None or from_id == to_id:
            return
        from_wp = self.waypoints.get(from_id)
        to_wp = self.waypoints.get(to_id)
        if from_wp is None or to_wp is None:
            return
        if 'lat' not in from_wp or 'lat' not in to_wp:
            return
        dist = haversine(from_wp['lat'], from_wp['lon'],
                         to_wp['lat'], to_wp['lon'])
        for edge in self.edges:
            if ((edge['from'] == from_id and edge['to'] == to_id)
                    or (edge['from'] == to_id and edge['to'] == from_id)):
                edge['distance'] = round(dist, 2)
                return
        self.edges.append({
            'from': from_id,
            'to': to_id,
            'distance': round(dist, 2),
        })

    # === 持久化 ===

    def _load(self):
        if not os.path.exists(self.data_file):
            self.get_logger().info('无已有路径图，从零开始')
            return
        try:
            with open(self.data_file, 'r') as f:
                data = json.load(f)
            self.waypoints = data.get('waypoints', {})
            self.edges = data.get('edges', [])
            self.wp_counter = data.get('wp_counter', 0)
            for wp_id in self.waypoints:
                m = re.match(r'wp_(\d+)$', wp_id)
                if m:
                    self.wp_counter = max(self.wp_counter, int(m.group(1)))
            self.get_logger().info(
                f'已加载路径图: {len(self.waypoints)} 路点, {len(self.edges)} 边')
        except Exception as e:
            self.get_logger().error(f'加载路径图失败: {e}')
            backup = self.data_file + f'.bak.{int(time.time())}'
            try:
                os.rename(self.data_file, backup)
            except OSError:
                pass

    def _save(self):
        if not self.dirty:
            return
        try:
            dir_name = os.path.dirname(self.data_file)
            if dir_name:
                os.makedirs(dir_name, exist_ok=True)
            data = {
                'waypoints': self.waypoints,
                'edges': self.edges,
                'wp_counter': self.wp_counter,
            }
            tmp_file = self.data_file + '.tmp'
            with open(tmp_file, 'w') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            os.replace(tmp_file, self.data_file)
            self.dirty = False
            self.get_logger().info(
                f'路径图已保存: {len(self.waypoints)} 路点, {len(self.edges)} 边')
        except Exception as e:
            self.get_logger().error(f'保存路径图失败: {e}')

    def _stats_timer_cb(self):
        total_dist = sum(e.get('distance', 0.0) for e in self.edges)
        labeled = sum(1 for wp in self.waypoints.values() if wp.get('label'))
        stats = {
            'waypoint_count': len(self.waypoints),
            'edge_count': len(self.edges),
            'total_distance_m': round(total_dist, 1),
            'labeled_count': labeled,
        }
        msg = String()
        msg.data = json.dumps(stats)
        self.stats_pub.publish(msg)

    def _auto_save_cb(self):
        self._save()


def main(args=None):
    rclpy.init(args=args)
    node = GpsPathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._save()
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
