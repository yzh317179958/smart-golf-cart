"""P2-5 GPS 召唤服务

接收召唤请求（来自 MQTT 或 CLI），解析后发送到 /nav_trigger。

支持两种召唤方式：
1. 球洞名: {"target": "3号洞"} → 直接转发到 /nav_trigger
2. GPS 坐标: {"lat": 22.66, "lon": 114.22} → 查路径图找最近路点 → /nav_trigger

到达后（/nav_complete）→ 切回 FOLLOWING 模式。
"""

import json
import math
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def haversine(lat1, lon1, lat2, lon2):
    R = 6371000.0
    rlat1, rlat2 = math.radians(lat1), math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


class SummonService(Node):

    def __init__(self):
        super().__init__('summon_service')

        # 参数
        self.declare_parameter(
            'data_file',
            os.path.expanduser('~/golf_ws/data/production/path_graph.json'))
        self.data_file = self.get_parameter('data_file').value

        # 路径图缓存
        self.waypoints = {}

        # 召唤状态（防导航完成误报）
        self._active_target = None

        # 订阅召唤请求（来自 MQTT bridge）
        self.create_subscription(
            String, '/summon_request', self._summon_cb, 10)

        # 订阅导航完成（来自 gps_waypoint_follower）
        self.create_subscription(
            String, '/nav_complete', self._nav_complete_cb, 10)

        # 发布
        self.nav_trigger_pub = self.create_publisher(String, '/nav_trigger', 10)
        self.summon_result_pub = self.create_publisher(
            String, '/summon_result', 10)

        self.get_logger().info('Summon service started')

    def _load_graph(self):
        """加载路径图（每次召唤时重新加载，保证最新数据）"""
        if not os.path.exists(self.data_file):
            self.get_logger().warning(f'路径图不存在: {self.data_file}')
            self.waypoints = {}
            return
        try:
            with open(self.data_file, 'r') as f:
                data = json.load(f)
            self.waypoints = data.get('waypoints', {})
        except Exception as e:
            self.get_logger().error(f'加载路径图失败: {e}')
            # 保持上次成功加载的缓存，不清空

    def _summon_cb(self, msg: String):
        """处理召唤请求 JSON"""
        try:
            req = json.loads(msg.data)
        except json.JSONDecodeError:
            # 纯文本当作球洞名
            req = {'target': msg.data}

        self.get_logger().info(f'收到召唤请求: {req}')

        # 方式1：球洞名/路点ID
        if 'target' in req:
            target = str(req['target']).strip()[:64]
            if not target:
                self._publish_result('error', '目标为空')
                return
            self._send_nav(target)
            return

        # 方式2：GPS 坐标（"来找我"）
        raw_lat = req.get('lat')
        raw_lon = req.get('lon')
        if raw_lat is not None and raw_lon is not None:
            try:
                lat = float(raw_lat)
                lon = float(raw_lon)
            except (TypeError, ValueError):
                self._publish_result('error', '坐标格式错误')
                return
            if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                self._publish_result('error', f'坐标超出范围: {lat}, {lon}')
                return
            self._summon_by_gps(lat, lon)
            return

        self._publish_result('error', '无效请求格式')

    def _summon_by_gps(self, lat, lon):
        """根据手机 GPS 坐标找最近路点，发起导航"""
        self._load_graph()
        if not self.waypoints:
            self._publish_result('error', '无路点数据')
            return

        best_id = None
        best_dist = float('inf')
        for wp_id, wp in self.waypoints.items():
            wp_lat = wp.get('lat')
            wp_lon = wp.get('lon')
            if wp_lat is None or wp_lon is None:
                continue
            d = haversine(lat, lon, wp_lat, wp_lon)
            if d < best_dist:
                best_id = wp_id
                best_dist = d

        if best_id is None:
            self._publish_result('error', '无法找到最近路点')
            return

        self.get_logger().info(
            f'GPS召唤: 手机({lat:.6f}, {lon:.6f}) → '
            f'最近路点 {best_id} (距离 {best_dist:.1f}m)')
        self._send_nav(best_id)

    def _send_nav(self, target):
        """发送导航请求到 gps_waypoint_follower"""
        self._active_target = target
        msg = String()
        msg.data = target
        self.nav_trigger_pub.publish(msg)
        self._publish_result('navigating', f'正在导航到 {target}')

    def _nav_complete_cb(self, msg: String):
        """导航完成回调（仅在有活跃召唤时响应）"""
        if self._active_target is None:
            return
        self.get_logger().info(f'导航完成: {msg.data}')
        self._active_target = None
        self._publish_result('arrived', msg.data)

    def _publish_result(self, status, detail=''):
        """发布召唤结果（MQTT bridge 会转发到 APP）"""
        result = json.dumps({
            'status': status,
            'detail': detail,
        }, ensure_ascii=False)
        msg = String()
        msg.data = result
        self.summon_result_pub.publish(msg)
        self.get_logger().info(f'召唤结果: {result}')


def main(args=None):
    rclpy.init(args=args)
    node = SummonService()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
