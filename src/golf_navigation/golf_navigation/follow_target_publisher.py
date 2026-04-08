"""跟随控制器（v8.1: MPPI 避障模式 + PID 回退模式）

MPPI 模式 (use_mppi=true, 默认):
  YOLO bbox+深度 → 计算人位置 → FollowPath action → MPPI 避障跟随
  controller_server 输出 /cmd_vel_nav → LiDAR 急停 → /cmd_vel

PID 回退模式 (use_mppi=false):
  YOLO bbox+深度 → PID → /cmd_vel（直发，v2.6.0 行为）

PID 类完全复制自 WheelTec simple_follower_ros2/visualFollower.py。
"""

import math
import numpy as np
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from yolo_msgs.msg import Detection

# MPPI 模式需要
try:
    from nav2_msgs.action import FollowPath
    from rclpy.action import ActionClient
    import tf2_ros
    _NAV2_AVAILABLE = True
except ImportError:
    _NAV2_AVAILABLE = False


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

        # === MPPI 模式 ===
        self.declare_parameter('use_mppi', False)
        self.use_mppi = self.get_parameter('use_mppi').value and _NAV2_AVAILABLE

        if self.use_mppi:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')
            self.mppi_ready = False
            self._goal_handle = None  # 保存 goal handle，用于取消
            self.last_goal_time = 0.0
            self.last_goal_x = 0.0
            self.last_goal_y = 0.0
            # MPPI 模式不直接发 cmd_vel（controller_server 发）
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_unused', 10)
            output_label = 'MPPI FollowPath → controller_server → /cmd_vel_nav'
        else:
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            output_label = '/cmd_vel (PID直发)'

        # WheelTec 原版：1 秒无数据停车
        self.last_target_time = time.monotonic()
        self.create_timer(0.5, self.safety_timer_cb)

        self.get_logger().info(
            f'Follow controller | mode: {"MPPI避障" if self.use_mppi else "PID直发"} '
            f'| target: {target_dist}mm | max_speed: {self.max_speed} '
            f'| output: {output_label}')

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

        # === MPPI 模式: 发送 FollowPath 给 controller_server ===
        if self.use_mppi:
            self._send_mppi_goal(angle_x, distance)
            return

        # === PID 模式: WheelTec 原版控制逻辑 ===

        # PID 更新
        [uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angle_x, distance])

        # 限幅（WheelTec 原版 + 禁止后退）
        angular_speed = np.clip(-uncliped_ang_speed, -self.max_speed, self.max_speed)
        linear_speed = np.clip(-uncliped_lin_speed, 0.0, self.max_speed)  # 禁止后退

        # 目标比期望距离近时，冻结线速度积分器（防止负积分累积导致响应迟滞）
        if linear_speed == 0.0:
            self.PID_controller.integrator[1] = 0.0

        # WheelTec 原版：超出范围 → 停车
        # 测试阶段无 collision_monitor，直发 /cmd_vel；Phase 4 启用后改回 /cmd_vel_raw
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
        """停车: PID 模式发零速度, MPPI 模式取消 FollowPath goal"""
        if self.use_mppi and self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._goal_handle = None
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.cmd_pub.publish(velocity)

    # === MPPI 避障跟随 ===

    def _send_mppi_goal(self, angle_x, distance_mm):
        """计算人在 odom 帧位置，发送 FollowPath 给 MPPI controller_server"""
        # 等待 action server 就绪
        if not self.mppi_ready:
            if not self.follow_path_client.server_is_ready():
                self.get_logger().info('等待 MPPI controller_server...', throttle_duration_sec=3.0)
                return
            self.mppi_ready = True
            self.get_logger().info('MPPI controller_server 就绪')

        distance_m = distance_mm / 1000.0

        # 人在 base_footprint 帧的位置（x前 y左）
        person_x_base = distance_m * math.cos(angle_x)
        person_y_base = distance_m * math.sin(angle_x)

        # TF 转换: base_footprint → odom_combined
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom_combined', 'base_footprint', rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        # 手动转换（避免 tf2_geometry_msgs 依赖问题）
        # 从 TF 取 robot 在 odom 帧的位置和航向
        robot_x = tf.transform.translation.x
        robot_y = tf.transform.translation.y
        # 四元数→yaw
        q = tf.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        robot_yaw = math.atan2(siny, cosy)

        # 旋转 base→odom
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        person_x_odom = robot_x + person_x_base * cos_yaw - person_y_base * sin_yaw
        person_y_odom = robot_y + person_x_base * sin_yaw + person_y_base * cos_yaw

        # 节流：人移动 >0.5m 或 >1s 才更新
        dx = person_x_odom - self.last_goal_x
        dy = person_y_odom - self.last_goal_y
        dist_moved = math.sqrt(dx * dx + dy * dy)
        now = time.monotonic()
        if dist_moved < 0.5 and now - self.last_goal_time < 1.0:
            return

        # 创建 2 点路径: robot → person
        path = Path()
        path.header.frame_id = 'odom_combined'
        path.header.stamp = self.get_clock().now().to_msg()

        start = PoseStamped()
        start.header = path.header
        start.pose.position.x = robot_x
        start.pose.position.y = robot_y
        start.pose.orientation.w = 1.0
        path.poses.append(start)

        goal = PoseStamped()
        goal.header = path.header
        goal.pose.position.x = person_x_odom
        goal.pose.position.y = person_y_odom
        goal.pose.orientation.w = 1.0
        path.poses.append(goal)

        # 取消旧 goal + 发送新 FollowPath
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        future = self.follow_path_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_cb)

        self.last_goal_x = person_x_odom
        self.last_goal_y = person_y_odom
        self.last_goal_time = now

        self.get_logger().info(
            f'MPPI goal: person({person_x_odom:.1f},{person_y_odom:.1f}) '
            f'dist={distance_m:.1f}m angle={math.degrees(angle_x):.0f}°',
            throttle_duration_sec=1.0)

    def _goal_response_cb(self, future):
        """保存 goal handle 用于后续取消"""
        try:
            self._goal_handle = future.result()
        except Exception:
            self._goal_handle = None


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
