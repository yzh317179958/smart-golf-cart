"""P2-17 测试日志记录器

实车测试时后台运行，记录 GPS 轨迹 + 系统事件到 JSON 文件。
测试后拷贝到 VPS，用 generate_test_map.py 生成可视化 HTML。

用法（WheelTec 上）:
    ros2 run golf_control test_logger
    # Ctrl+C 停止，自动保存

输出: ~/golf_ws/data/test_log_YYYYMMDD_HHMMSS.json
"""

import json
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Float32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist


class TestLogger(Node):

    def __init__(self):
        super().__init__('test_logger')

        self.declare_parameter('data_dir',
                               os.path.expanduser('~/golf_ws/data'))
        data_dir = self.get_parameter('data_dir').value
        os.makedirs(data_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file = os.path.join(data_dir, f'test_log_{timestamp}.json')

        # 数据缓冲
        self.start_time = time.monotonic()
        self.start_datetime = datetime.now().strftime('%Y-%m-%dT%H:%M:%S')
        self.trajectory = []    # GPS 轨迹点
        self.events = []        # 系统事件

        # 当前状态（附加到每个轨迹点）
        self.current_mode = 'unknown'
        self.current_heading = 0.0
        self.current_speed = 0.0
        self.current_angular = 0.0

        # 采样节流
        self.last_gps_time = 0.0
        self.gps_interval = 0.2   # 5Hz

        # 事件去重（只记状态变化）
        self.prev_mode = ''
        self.prev_follow = ''
        # 订阅
        gps_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(NavSatFix, '/gps/fix', self._gps_cb, gps_qos)
        self.create_subscription(Float32, '/heading_deg', self._heading_cb, gps_qos)
        self.create_subscription(Twist, '/cmd_vel', self._vel_cb, 10)
        self.create_subscription(String, '/system_mode', self._mode_cb, 10)
        self.create_subscription(String, '/follow_state', self._follow_cb, 10)
        self.create_subscription(String, '/nav_trigger', self._nav_trigger_cb, 10)
        self.create_subscription(String, '/nav_complete', self._nav_complete_cb, 10)

        # 定时保存（每 60 秒）
        self.create_timer(60.0, self._auto_save)

        self.get_logger().info(
            f'测试日志记录器启动 | 输出: {self.log_file}')

    def _t(self):
        """相对时间戳（秒）"""
        return round(time.monotonic() - self.start_time, 2)

    # === GPS 轨迹（5Hz 采样） ===

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status < 0 or (msg.latitude == 0.0 and msg.longitude == 0.0):
            return
        now = time.monotonic()
        if now - self.last_gps_time < self.gps_interval:
            return
        self.last_gps_time = now

        self.trajectory.append({
            't': self._t(),
            'lat': round(msg.latitude, 8),
            'lon': round(msg.longitude, 8),
            'heading': round(self.current_heading, 1),
            'speed': round(self.current_speed, 3),
            'angular': round(self.current_angular, 3),
            'mode': self.current_mode,
        })

    def _heading_cb(self, msg: Float32):
        self.current_heading = msg.data

    def _vel_cb(self, msg: Twist):
        self.current_speed = msg.linear.x
        self.current_angular = msg.angular.z

    # === 事件（仅状态变化） ===

    def _mode_cb(self, msg: String):
        self.current_mode = msg.data
        if msg.data != self.prev_mode:
            self.prev_mode = msg.data
            self.events.append({
                't': self._t(), 'type': 'mode_change', 'data': msg.data})

    def _follow_cb(self, msg: String):
        if msg.data != self.prev_follow:
            self.prev_follow = msg.data
            self.events.append({
                't': self._t(), 'type': 'follow_state', 'data': msg.data})

    def _nav_trigger_cb(self, msg: String):
        self.events.append({
            't': self._t(), 'type': 'nav_trigger', 'data': msg.data})

    def _nav_complete_cb(self, msg: String):
        self.events.append({
            't': self._t(), 'type': 'nav_complete', 'data': msg.data})

    # === 保存 ===

    def _auto_save(self):
        self._save()
        self.get_logger().info(
            f'自动保存 | 轨迹点: {len(self.trajectory)} | 事件: {len(self.events)}',
            throttle_duration_sec=60.0)

    def _save(self):
        data = {
            'start_time': self.start_datetime,
            'duration_sec': round(self._t(), 1),
            'gps_trajectory': self.trajectory,
            'events': self.events,
        }
        try:
            tmp = self.log_file + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(data, f, ensure_ascii=False)
            os.replace(tmp, self.log_file)
        except Exception as e:
            self.get_logger().error(f'保存日志失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TestLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node._save()
            node.get_logger().info(
                f'日志已保存: {len(node.trajectory)} 轨迹点, '
                f'{len(node.events)} 事件 → {node.log_file}')
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
