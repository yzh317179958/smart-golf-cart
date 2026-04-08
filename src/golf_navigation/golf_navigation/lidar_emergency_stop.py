"""P2-11 LiDAR 急停保护层（仅导航模式，方案A）

门控 /cmd_vel_nav → /cmd_vel，前方障碍物检测：
  >3m  → 透传（正常行驶）
  2-3m → 减速 50%
  <2m  → 停车等待
  停 30s → 发布 /nav_blocked（通知 follower 触发脱困）

仅在导航模式生效。跟随模式的 /cmd_vel 不经过本节点（方案A架构）。
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# 状态
_PASSING = 'passing'
_SLOWDOWN = 'slowdown'
_STOPPED = 'stopped'


class LidarEmergencyStop(Node):

    def __init__(self):
        super().__init__('lidar_emergency_stop')

        # === 参数 ===
        self.declare_parameter('stop_distance', 2.0)        # 停车距离 (m)
        self.declare_parameter('slowdown_distance', 3.0)     # 减速距离 (m)
        self.declare_parameter('slowdown_ratio', 0.5)        # 减速比例
        self.declare_parameter('front_angle', 30.0)          # 前方检测半角 (°)
        self.declare_parameter('clear_time', 2.0)            # 障碍消失确认时间 (s)
        self.declare_parameter('block_timeout', 30.0)        # 停车超时 → 通知 (s)
        self.declare_parameter('cmd_vel_in', '/cmd_vel_nav')
        self.declare_parameter('cmd_vel_out', '/cmd_vel')

        self.stop_dist = self.get_parameter('stop_distance').value
        self.slow_dist = self.get_parameter('slowdown_distance').value
        self.slow_ratio = self.get_parameter('slowdown_ratio').value
        self.front_angle_rad = math.radians(self.get_parameter('front_angle').value)
        self.clear_time = self.get_parameter('clear_time').value
        self.block_timeout = self.get_parameter('block_timeout').value
        cmd_in = self.get_parameter('cmd_vel_in').value
        cmd_out = self.get_parameter('cmd_vel_out').value

        # === 状态 ===
        self.state = _PASSING
        self.stop_start_time = 0.0        # 停车开始时间
        self.clear_start_time = 0.0       # 障碍消失开始时间
        self.block_notified = False       # 是否已发过 blocked 通知
        self.min_front_dist = float('inf')  # 前方最近障碍物距离
        self.min_left_dist = float('inf')   # 左侧最近距离（SLOWDOWN 偏转用）
        self.min_right_dist = float('inf')  # 右侧最近距离
        self.last_cmd_nav = Twist()       # 最新的导航速度指令

        # === 订阅 ===
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, scan_qos)
        self.create_subscription(Twist, cmd_in, self._cmd_nav_cb, 10)

        # === 发布 ===
        self.cmd_pub = self.create_publisher(Twist, cmd_out, 10)
        self.blocked_pub = self.create_publisher(String, '/nav_blocked', 10)
        self.state_pub = self.create_publisher(String, '/emergency_stop_state', 10)

        # === 定时器 ===
        self.create_timer(0.05, self._control_loop)  # 20Hz 门控循环

        self.get_logger().info(
            f'LiDAR 急停启动 | 停车: <{self.stop_dist}m | '
            f'减速: <{self.slow_dist}m | 前方: ±{math.degrees(self.front_angle_rad):.0f}° | '
            f'{cmd_in} → {cmd_out}')

    # ================================================================
    #  回调
    # ================================================================

    def _scan_cb(self, msg: LaserScan):
        """从 /scan 提取前方扇形区域最近障碍物距离 + 左右分区"""
        min_dist = float('inf')
        min_left = float('inf')    # 前方左半区 (angle > 0)
        min_right = float('inf')   # 前方右半区 (angle < 0)

        for i, r in enumerate(msg.ranges):
            # 跳过无效值
            if r <= msg.range_min or r >= msg.range_max or math.isinf(r) or math.isnan(r):
                continue

            # 计算该射线的角度
            angle = msg.angle_min + i * msg.angle_increment

            # 归一化到 [-pi, pi]
            angle = (angle + math.pi) % (2.0 * math.pi) - math.pi

            # 只看前方扇形区域
            if abs(angle) <= self.front_angle_rad:
                if r < min_dist:
                    min_dist = r
                # 分左右记录（用于 SLOWDOWN 偏转）
                if angle > 0.0 and r < min_left:
                    min_left = r
                elif angle <= 0.0 and r < min_right:
                    min_right = r

        self.min_front_dist = min_dist
        self.min_left_dist = min_left
        self.min_right_dist = min_right

    def _cmd_nav_cb(self, msg: Twist):
        """缓存最新的导航速度指令"""
        self.last_cmd_nav = msg

    # ================================================================
    #  门控逻辑（20Hz）
    # ================================================================

    def _control_loop(self):
        now = time.monotonic()
        dist = self.min_front_dist
        obstacle_near = dist < self.stop_dist  # 恢复阈值=停车阈值，clear_time 防抖动

        # === 状态转换 ===

        if self.state == _PASSING:
            if dist < self.stop_dist:
                self.state = _STOPPED
                self.stop_start_time = now
                self.block_notified = False
                self.get_logger().warn(
                    f'前方障碍 {dist:.1f}m < {self.stop_dist}m → 停车')
            elif dist < self.slow_dist:
                self.state = _SLOWDOWN
                self.get_logger().info(
                    f'前方障碍 {dist:.1f}m < {self.slow_dist}m → 减速',
                    throttle_duration_sec=3.0)

        elif self.state == _SLOWDOWN:
            if dist < self.stop_dist:
                self.state = _STOPPED
                self.stop_start_time = now
                self.block_notified = False
                self.get_logger().warn(
                    f'前方障碍 {dist:.1f}m → 停车')
            elif dist >= self.slow_dist:
                self.state = _PASSING

        elif self.state == _STOPPED:
            if not obstacle_near:
                # 障碍消失，开始确认计时
                if self.clear_start_time == 0.0:
                    self.clear_start_time = now
                elif now - self.clear_start_time >= self.clear_time:
                    self.state = _PASSING
                    self.clear_start_time = 0.0
                    self.get_logger().info('障碍消失，恢复行驶')
            else:
                self.clear_start_time = 0.0  # 障碍还在，重置计时

                # 停车超时通知
                if not self.block_notified and now - self.stop_start_time >= self.block_timeout:
                    self.block_notified = True
                    msg = String()
                    msg.data = f'front_obstacle_{dist:.1f}m'
                    self.blocked_pub.publish(msg)
                    self.get_logger().error(
                        f'停车超时 {self.block_timeout:.0f}s → 发布 /nav_blocked')

        # === 输出速度 ===

        cmd = Twist()

        if self.state == _PASSING:
            cmd = self.last_cmd_nav

        elif self.state == _SLOWDOWN:
            cmd.linear.x = self.last_cmd_nav.linear.x * self.slow_ratio
            cmd.angular.z = self.last_cmd_nav.angular.z * self.slow_ratio
            # 避障微偏转：左侧更近→偏右(负), 右侧更近→偏左(正), 两侧差不多→不偏
            left_d = self.min_left_dist
            right_d = self.min_right_dist
            if left_d < self.slow_dist and right_d < self.slow_dist:
                diff = right_d - left_d  # 正=右远左近→偏右
                if abs(diff) > 0.3:      # 差异>0.3m 才偏转，走廊两侧差不多不动
                    nudge = -0.15 if diff > 0 else 0.15
                    cmd.angular.z += nudge

        elif self.state == _STOPPED:
            # 阿克曼底盘不能原地转，慢速爬行+转弯避开障碍
            left_d = self.min_left_dist
            right_d = self.min_right_dist
            cmd.linear.x = 0.1  # 最低爬行速度
            if left_d < right_d:
                cmd.angular.z = -0.4  # 左近→右转
            else:
                cmd.angular.z = 0.4   # 右近→左转

        self.cmd_pub.publish(cmd)

        # 发布状态
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarEmergencyStop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # 退出时发零速度
        try:
            node.cmd_pub.publish(Twist())
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
