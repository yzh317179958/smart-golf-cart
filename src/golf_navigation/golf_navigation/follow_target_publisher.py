"""跟随控制器

YOLO bbox + 深度 → simplePID → /cmd_vel，直发底盘。

simplePID 类完全复制自 WheelTec simple_follower_ros2/visualFollower.py，
目标距离默认 600mm，超过 out_of_range 距离自动停车。
"""

import numpy as np
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from yolo_msgs.msg import Detection


# ============================================================
# 以下 simplePID 类完全复制自 WheelTec visualFollower.py，未做任何修改
# ============================================================
class simplePID:
    '''very simple discrete PID controller'''
    def __init__(self, target, P, I, D):
        if(not(np.size(P)==np.size(I)==np.size(D)) or ((np.size(target)==1) and np.size(P)!=1) or (np.size(target)!=1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
            raise TypeError('input parameters shape is not compatable')

        self.Kp     = np.array(P)
        self.Ki     = np.array(I)
        self.Kd     = np.array(D)
        self.setPoint = np.array(target)

        self.last_error = 0
        self.integrator = 0
        self.integrator_max = 500.0
        self.timeOfLastCall = None

    def update(self, current_value):
        current_value = np.array(current_value)
        if(np.size(current_value) != np.size(self.setPoint)):
            raise TypeError('current_value and target do not have the same shape')
        if(self.timeOfLastCall is None):
            self.timeOfLastCall = time.perf_counter()
            return np.zeros(np.size(current_value))

        error = self.setPoint - current_value

        # WheelTec 原版死区：角度 <0.1rad 归零，距离 <150mm 归零
        if error[0] < 0.1 and error[0] > -0.1:
            error[0] = 0
        if error[1] < 150 and error[1] > -150:
            error[1] = 0

        # WheelTec 原版：目标近时放大速度
        if (error[1] > 0 and self.setPoint[1] < 1200):
            error[1] = error[1] * (1200 / self.setPoint[1]) * 0.5
            error[0] = error[0] * 0.8

        P = error

        currentTime = time.perf_counter()
        deltaT = (currentTime - self.timeOfLastCall)

        self.integrator = np.clip(
            self.integrator + (error * deltaT),
            -self.integrator_max, self.integrator_max)
        I = self.integrator

        D = (error - self.last_error) / deltaT

        self.last_error = error
        self.timeOfLastCall = currentTime

        return self.Kp * P + self.Ki * I + self.Kd * D
# ============================================================


class FollowTargetPublisher(Node):

    def __init__(self):
        super().__init__('follow_target_publisher')

        # 参数（WheelTec 原版值）
        self.declare_parameter('target_distance', 600)   # mm（WheelTec 原版）
        self.declare_parameter('max_speed', 0.3)         # m/s（WheelTec 原版）
        self.declare_parameter('angular_kp', 1.2)        # WheelTec 原版
        self.declare_parameter('angular_ki', 0.0)        # WheelTec 原版
        self.declare_parameter('angular_kd', 0.005)      # WheelTec 原版
        self.declare_parameter('linear_kp', 0.2)         # WheelTec 原版
        self.declare_parameter('linear_ki', 0.0)         # WheelTec 原版
        self.declare_parameter('linear_kd', 0.0)         # WheelTec 原版
        self.declare_parameter('out_of_range', 2000)     # mm（WheelTec 原版：>2000 停车）
        self.declare_parameter('min_depth', 300)         # mm
        # S11 水平半 FOV 的正切值（S11 约 120° FOV → 半 FOV 60° → tan(60°) = 1.7321）
        # 对齐 WheelTec visualTracker.py: angle = -arctan(displacement * tanHorizontal)
        self.declare_parameter('tan_half_fov', 1.7321)

        target_dist = self.get_parameter('target_distance').value
        self.max_speed = self.get_parameter('max_speed').value
        self.out_of_range = self.get_parameter('out_of_range').value
        self.min_depth = self.get_parameter('min_depth').value
        self.tan_half_fov = self.get_parameter('tan_half_fov').value

        ang_kp = self.get_parameter('angular_kp').value
        ang_ki = self.get_parameter('angular_ki').value
        ang_kd = self.get_parameter('angular_kd').value
        lin_kp = self.get_parameter('linear_kp').value
        lin_ki = self.get_parameter('linear_ki').value
        lin_kd = self.get_parameter('linear_kd').value

        # WheelTec 原版 PID：target=[0度, 目标距离mm]
        self.PID_controller = simplePID(
            [0, target_dist],
            [ang_kp, lin_kp],
            [ang_ki, lin_ki],
            [ang_kd, lin_kd]
        )

        # S11 相机分辨率
        self.rgb_w, self.rgb_h = 1280, 1080
        self.depth_w, self.depth_h = 240, 90

        # 状态
        self.follow_state = 'idle'
        self.system_mode = 'following'  # 系统模式：following / navigation / e_stop
        self.depth_image = None

        # 订阅
        self.create_subscription(String, '/follow_state', self.state_cb, 10)
        self.create_subscription(String, '/system_mode', self._system_mode_cb, 10)
        self.create_subscription(Image, '/lx_camera_node/LxCamera_Depth', self.depth_cb, 5)
        self.create_subscription(Detection, '/locked_target', self.target_cb, 10)

        # 发布: PID 直发底盘速度
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # WheelTec 原版：1 秒无数据停车
        self.last_target_time = time.monotonic()
        self.create_timer(0.5, self.safety_timer_cb)

        self.get_logger().info(
            f'Follow controller | target: {target_dist}mm '
            f'| max_speed: {self.max_speed} | output: /cmd_vel')

    def _system_mode_cb(self, msg: String):
        new_mode = msg.data.lower()
        if new_mode != self.system_mode:
            self.get_logger().info(f'系统模式切换: {self.system_mode} -> {new_mode}')
            if new_mode != 'following':
                self.stop_moving()
            self.system_mode = new_mode

    def state_cb(self, msg: String):
        prev = self.follow_state
        self.follow_state = msg.data
        if prev == 'tracking' and self.follow_state != 'tracking':
            self.stop_moving()

    def depth_cb(self, msg: Image):
        if msg.encoding not in ('16UC1', 'mono16'):
            return
        self.depth_image = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.width)

    def target_cb(self, msg: Detection):
        if self.system_mode != 'following':
            return
        if self.follow_state != 'tracking':
            return
        if self.depth_image is None:
            return

        self.last_target_time = time.monotonic()

        # --- 从 bbox 计算角度（复制自 WheelTec visualTracker.py）---
        rgb_cx = msg.bbox.center.position.x
        rgb_cy = msg.bbox.center.position.y
        displacement = 2.0 * rgb_cx / self.rgb_w - 1.0  # [-1, 1]
        angle_x = -1.0 * np.arctan(displacement * self.tan_half_fov)  # 弧度

        # --- 从深度图计算距离（适配输入源，逻辑等价于 WheelTec distance）---
        depth_x = int(rgb_cx * self.depth_w / self.rgb_w)
        depth_y = int(rgb_cy * self.depth_h / self.rgb_h)
        depth_x = max(0, min(depth_x, self.depth_w - 1))
        depth_y = max(0, min(depth_y, self.depth_h - 1))

        r = 2
        y0, y1 = max(0, depth_y - r), min(self.depth_h, depth_y + r + 1)
        x0, x1 = max(0, depth_x - r), min(self.depth_w, depth_x + r + 1)
        patch = self.depth_image[y0:y1, x0:x1].flatten()
        valid = patch[(patch > self.min_depth) & (patch < self.out_of_range)]
        if len(valid) == 0:
            distance = 0
        else:
            distance = float(np.median(valid))

        # 深度无效时停车，不调用 PID（防止积分器被 0 污染）
        if distance == 0:
            self.stop_moving()
            return

        # PID 更新（WheelTec 原版控制逻辑）
        [uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angle_x, distance])

        # 限幅（WheelTec 原版 + 禁止后退）
        angular_speed = np.clip(-uncliped_ang_speed, -self.max_speed, self.max_speed)
        linear_speed = np.clip(-uncliped_lin_speed, 0.0, self.max_speed)  # 禁止后退

        # 目标比期望距离近时，冻结线速度积分器（防止负积分累积导致响应迟滞）
        if linear_speed == 0.0:
            self.PID_controller.integrator[1] = 0.0

        # WheelTec 原版：超出范围 → 停车
        if distance > self.out_of_range:
            self.stop_moving()
        else:
            velocity = Twist()
            velocity.linear.x = float(linear_speed)
            velocity.angular.z = float(angular_speed)
            self.cmd_pub.publish(velocity)

        self.get_logger().info(
            f'dist: {distance:.0f}mm | angle: {angle_x:.2f} | '
            f'v: {linear_speed:.2f} w: {angular_speed:.2f}',
            throttle_duration_sec=1.0)

    def safety_timer_cb(self):
        """WheelTec 原版 controllerLoss 等价"""
        if self.follow_state == 'tracking':
            if time.monotonic() - self.last_target_time > 1.0:
                self.stop_moving()

    def stop_moving(self):
        """停车：发零速度"""
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.cmd_pub.publish(velocity)


def main(args=None):
    rclpy.init(args=args)
    node = FollowTargetPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.stop_moving()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
