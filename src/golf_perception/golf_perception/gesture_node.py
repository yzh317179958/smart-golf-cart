"""手势识别节点：基于 YOLO Pose 关键点判定举手手势.

右手举起 → lock（开始/停止切换）
左手举起 → stop（直接停止）
按关键点 id 查找（稀疏数组安全）。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from yolo_msgs.msg import DetectionArray
import time


class GestureNode(Node):

    def __init__(self):
        super().__init__('gesture_node')

        self.declare_parameter('threshold', 15.0)
        self.declare_parameter('lock_min_frames', 3)
        self.declare_parameter('stop_min_frames', 4)
        self.declare_parameter('keypoint_confidence', 0.5)
        self.declare_parameter('hold_duration', 2.0)
        self.declare_parameter('drop_tolerance', 5)

        self.threshold = self.get_parameter('threshold').value
        self.lock_min_frames = self.get_parameter('lock_min_frames').value
        self.stop_min_frames = self.get_parameter('stop_min_frames').value
        self.kp_conf = self.get_parameter('keypoint_confidence').value
        self.hold_duration = self.get_parameter('hold_duration').value
        self.drop_tolerance = self.get_parameter('drop_tolerance').value

        # 状态跟踪
        self.right_hand_start = None
        self.left_hand_start = None
        self.right_hand_frames = 0
        self.left_hand_frames = 0
        self.right_drop_count = 0
        self.left_drop_count = 0

        # 保持机制
        self.active_gesture = 'none'
        self.gesture_trigger_time = None

        self.pub = self.create_publisher(String, '/gesture_cmd', 10)
        self.create_subscription(
            DetectionArray, '/yolo/tracking', self.tracking_cb, 10)

        self.get_logger().info(
            f'Gesture node started (pose keypoints) | '
            f'threshold: {self.threshold} | lock: {self.lock_min_frames}f | '
            f'stop: {self.stop_min_frames}f | hold: {self.hold_duration}s')

    def tracking_cb(self, msg: DetectionArray):
        best = None
        best_area = 0.0
        for det in msg.detections:
            if det.class_name != 'person':
                continue
            area = det.bbox.size.x * det.bbox.size.y
            if area > best_area:
                best_area = area
                best = det

        right_up = False
        left_up = False

        if best is not None and len(best.keypoints.data) >= 4:
            kp_map = {kp.id: kp for kp in best.keypoints.data}
            ls = kp_map.get(5)   # left_shoulder
            rs = kp_map.get(6)   # right_shoulder
            lw = kp_map.get(9)   # left_wrist
            rw = kp_map.get(10)  # right_wrist

            if (rs and rw and rs.score >= self.kp_conf and rw.score >= self.kp_conf
                    and rw.point.y < rs.point.y - self.threshold):
                right_up = True

            if (ls and lw and ls.score >= self.kp_conf and lw.score >= self.kp_conf
                    and lw.point.y < ls.point.y - self.threshold):
                left_up = True

        now = time.monotonic()

        # 右手（lock）
        if right_up:
            self.right_drop_count = 0
            if self.right_hand_start is None:
                self.right_hand_start = now
                self.right_hand_frames = 0
            self.right_hand_frames += 1
        else:
            self.right_drop_count += 1
            if self.right_drop_count > self.drop_tolerance:
                self.right_hand_start = None
                self.right_hand_frames = 0

        right_triggered = (self.right_hand_frames >= self.lock_min_frames
                           and self.right_hand_start is not None)

        # 左手（stop）
        if left_up:
            self.left_drop_count = 0
            if self.left_hand_start is None:
                self.left_hand_start = now
                self.left_hand_frames = 0
            self.left_hand_frames += 1
        else:
            self.left_drop_count += 1
            if self.left_drop_count > self.drop_tolerance:
                self.left_hand_start = None
                self.left_hand_frames = 0

        left_triggered = (self.left_hand_frames >= self.stop_min_frames
                          and self.left_hand_start is not None)

        # 手势状态机
        if left_triggered:
            self.active_gesture = 'stop'
            self.gesture_trigger_time = now
        elif right_triggered:
            self.active_gesture = 'lock'
            self.gesture_trigger_time = now
        elif self.active_gesture != 'none':
            if (now - self.gesture_trigger_time) > self.hold_duration:
                self.active_gesture = 'none'

        out = String()
        out.data = self.active_gesture
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GestureNode()
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
