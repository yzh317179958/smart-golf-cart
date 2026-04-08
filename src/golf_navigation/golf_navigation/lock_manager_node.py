"""目标锁定节点：自动跟随最大人体，手势仅用于暂停

检测到人 → 自动锁定最大 bbox → tracking
手势 stop → paused（暂停跟随）
手势结束 → 自动恢复
丢失超时 → idle
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from yolo_msgs.msg import Detection, DetectionArray


class LockManagerNode(Node):
    def __init__(self):
        super().__init__('lock_manager')

        self.declare_parameter('lost_timeout', 10.0)
        self.lost_timeout = self.get_parameter('lost_timeout').value

        self.state = 'idle'  # idle / tracking / paused
        self.locked_id = None
        self.last_seen_time = None

        self.create_subscription(String, '/gesture_cmd', self.gesture_cb, 10)
        self.create_subscription(DetectionArray, '/yolo/tracking', self.tracking_cb, 10)

        self.state_pub = self.create_publisher(String, '/follow_state', 10)
        self.target_pub = self.create_publisher(Detection, '/locked_target', 10)

        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info(f'LockManager started (auto-lock) | lost_timeout: {self.lost_timeout}s')

    def gesture_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'stop' and self.state == 'tracking':
            self.state = 'paused'
            self.get_logger().info('gesture stop → paused')
        elif cmd == 'none' and self.state == 'paused':
            # stop 手势结束 → 恢复到 idle，下一帧检测到人自动 re-lock
            self.state = 'idle'
            self.locked_id = None
            self.get_logger().info('gesture released → idle (will auto-lock)')

    def tracking_cb(self, msg: DetectionArray):
        if self.state == 'paused':
            return

        now = self.get_clock().now()
        persons = [d for d in msg.detections if d.class_name == 'person']

        if not persons:
            # 无人检测到
            if self.state == 'tracking' and self.last_seen_time is not None:
                elapsed = (now - self.last_seen_time).nanoseconds / 1e9
                if elapsed > self.lost_timeout:
                    self.state = 'idle'
                    self.locked_id = None
                    self.last_seen_time = None
                    self.get_logger().warn('Target lost timeout → idle')
            return

        # 有人检测到
        if self.state == 'idle':
            # 自动锁定最大人体
            best = max(persons, key=lambda d: d.bbox.size.x * d.bbox.size.y)
            self.locked_id = best.id
            self.state = 'tracking'
            self.last_seen_time = now
            self.target_pub.publish(best)
            self.get_logger().info(f'Auto-locked track_id={self.locked_id}')
            return

        # state == 'tracking': 跟踪已锁定目标
        target = None
        for det in persons:
            if det.id == self.locked_id:
                target = det
                break

        if target is not None:
            self.last_seen_time = now
            self.target_pub.publish(target)
        else:
            # 锁定 ID 丢失，重新选最大人体（YOLO track ID 可能变化）
            best = max(persons, key=lambda d: d.bbox.size.x * d.bbox.size.y)
            self.locked_id = best.id
            self.last_seen_time = now
            self.target_pub.publish(best)

    def timer_cb(self):
        msg = String()
        # paused 对外发布 idle，让 follow_target_publisher 停车
        msg.data = 'idle' if self.state == 'paused' else self.state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LockManagerNode()
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
