"""P1-13 模式管理器：FOLLOWING / NAVIGATION / E_STOP 三状态机

设计理念：
- FOLLOWING 是默认模式（高尔夫球车核心功能是跟人）
- NAVIGATION 手动触发（APP/话题/Foxglove 发 /nav_trigger）
- E_STOP 由手柄急停或目标长时间丢失触发
- /system_mode 供所有下游节点感知当前系统状态
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class ModeManagerNode(Node):
    # 系统模式
    FOLLOWING = 'following'
    NAVIGATION = 'navigation'
    E_STOP = 'e_stop'

    def __init__(self):
        super().__init__('mode_manager')

        # 参数
        self.declare_parameter('lost_estop_timeout', 15.0)
        self.declare_parameter('estop_joy_button', 0)  # 手柄急停按钮索引
        self.declare_parameter('resume_joy_button', 1)  # 手柄恢复按钮索引
        self.lost_estop_timeout = self.get_parameter('lost_estop_timeout').value
        self.estop_joy_button = self.get_parameter('estop_joy_button').value
        self.resume_joy_button = self.get_parameter('resume_joy_button').value

        # 状态
        self.mode = self.FOLLOWING
        self.follow_state = 'idle'
        self.lost_start_time = None
        self.nav_target = None

        # 发布
        self.mode_pub = self.create_publisher(String, '/system_mode', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 订阅
        self.create_subscription(String, '/follow_state', self._follow_state_cb, 10)
        self.create_subscription(String, '/gesture_cmd', self._gesture_cb, 10)
        self.create_subscription(String, '/nav_trigger', self._nav_trigger_cb, 10)
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_subscription(String, '/nav_complete', self._nav_complete_cb, 10)

        # 定时发布 /system_mode
        self.create_timer(0.5, self._publish_mode)

        self.get_logger().info(f'Mode manager started, initial mode: {self.mode}')

    def _set_mode(self, new_mode, reason=''):
        if new_mode == self.mode:
            return
        old = self.mode
        self.mode = new_mode
        self.get_logger().info(f'Mode: {old} -> {new_mode} ({reason})')

        if new_mode == self.E_STOP:
            self._stop_vehicle()
        elif new_mode == self.NAVIGATION and old == self.FOLLOWING:
            self._stop_vehicle()  # 先停车再导航
        elif new_mode == self.FOLLOWING and old == self.NAVIGATION:
            self.nav_target = None

    def _stop_vehicle(self):
        self.cmd_vel_pub.publish(Twist())

    def _publish_mode(self):
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)
        self._check_lost_timeout()

    def _follow_state_cb(self, msg: String):
        prev = self.follow_state
        self.follow_state = msg.data

        if msg.data in ('tracking',):
            self.lost_start_time = None
        elif msg.data in ('lost_slow', 'lost_wait') and prev == 'tracking':
            self.lost_start_time = self.get_clock().now()
        elif msg.data == 'lost_reset':
            self.lost_start_time = None

    def _check_lost_timeout(self):
        """目标丢失超过阈值 → E_STOP"""
        if self.mode != self.FOLLOWING:
            return
        if self.lost_start_time is None:
            return
        elapsed = (self.get_clock().now() - self.lost_start_time).nanoseconds / 1e9
        if elapsed > self.lost_estop_timeout:
            self._set_mode(self.E_STOP, f'target lost {elapsed:.0f}s')
            self.lost_start_time = None

    def _gesture_cb(self, msg: String):
        if msg.data == 'lock' and self.mode == self.NAVIGATION:
            self._set_mode(self.FOLLOWING, 'lock gesture during navigation')

    def _nav_trigger_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        # 远程 E_STOP（来自 APP）
        if cmd == 'e_stop':
            self._set_mode(self.E_STOP, 'remote e_stop')
            return
        # 远程取消导航（来自 APP）
        if cmd == 'cancel':
            if self.mode == self.NAVIGATION:
                self._set_mode(self.FOLLOWING, 'remote cancel')
            return
        # 远程恢复跟随（从 E_STOP 或 NAVIGATION 恢复）
        if cmd == 'resume':
            self._set_mode(self.FOLLOWING, 'remote resume')
            return
        if self.mode == self.E_STOP:
            self.get_logger().warn('Cannot navigate in E_STOP mode')
            return
        self.nav_target = msg.data
        self._set_mode(self.NAVIGATION, f'nav_trigger: {msg.data}')

    def _nav_complete_cb(self, msg: String):
        if self.mode == self.NAVIGATION:
            self._set_mode(self.FOLLOWING, f'navigation complete: {msg.data}')

    def _joy_cb(self, msg: Joy):
        if len(msg.buttons) <= max(self.estop_joy_button, self.resume_joy_button):
            return
        if msg.buttons[self.estop_joy_button]:
            self._set_mode(self.E_STOP, 'joy emergency stop')
        elif msg.buttons[self.resume_joy_button] and self.mode == self.E_STOP:
            self._set_mode(self.FOLLOWING, 'joy resume')


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
