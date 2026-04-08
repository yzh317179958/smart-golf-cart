"""H30 IMU NED→ENU 坐标转换 + G90 双天线航向融合

YeSense H30 AHRS 输出 NED 四元数（驱动直接透传），robot_localization 期望 ENU。

本节点做：
  1. G90 双天线航向优先：精度 ±0.3°，静止可用，不受磁场干扰
  2. 无 G90 航向时回退 H30 磁力计 NED→ENU 转换
  3. 角速度 / 线加速度 FRD→FLU 轴变换

G90 双天线航向来源：
  - NMEA 驱动发布 /heading（QuaternionStamped）
  - compass heading 已在驱动配置中经 rotation=180° 修正（ANT1前+ANT2后）
  - 本节点将 compass heading 转换为 ENU yaw 后输出

H30 磁力计回退：
  - 当 G90 航向超时（>2s 未收到）时，使用 H30 磁力计 NED→ENU 转换
  - yaw_offset 参数为 NED→ENU 偏移（含安装偏差 + 磁偏角）
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped


class ImuNedToEnu(Node):

    def __init__(self):
        super().__init__('imu_ned_to_enu')

        # === H30 NED→ENU fallback offset ===
        self.declare_parameter('yaw_offset', -1.988)
        self.yaw_offset = self.get_parameter('yaw_offset').value

        # === G90 双天线航向状态 ===
        self.g90_enu_yaw = None      # 最新 G90 ENU yaw (rad)
        self.g90_stamp_ns = 0        # G90 航向时间戳 (nanoseconds)
        self.G90_TIMEOUT_NS = 2_000_000_000  # 2s 超时

        # 订阅 G90 双天线航向
        self.create_subscription(
            QuaternionStamped, '/heading', self._heading_cb, 10)

        # === IMU 转换（200Hz）===
        self.create_subscription(Imu, '/h30_imu_raw', self._imu_cb, 10)
        self.pub = self.create_publisher(Imu, '/h30_imu', 10)

        self.get_logger().info(
            f'NED→ENU started | G90 dual-antenna primary | '
            f'H30 fallback offset={math.degrees(self.yaw_offset):.1f}°')

    # === G90 双天线航向回调 ===

    def _heading_cb(self, msg: QuaternionStamped):
        """G90 双天线航向：compass heading quaternion → ENU yaw

        NMEA 驱动用 quaternion_from_euler(0, 0, radians(heading_deg)) 构造四元数，
        其中 heading_deg 是 compass heading（0=North, 90=East, 顺时针），
        已经过 rotation=180° 修正（补偿 ANT1前+ANT2后 安装方向）。

        ENU 约定：yaw=0 朝东，逆时针为正。
        转换公式：enu_yaw = π/2 - compass_yaw
        """
        q = msg.quaternion
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        compass_yaw = math.atan2(siny, cosy)

        enu_yaw = math.pi / 2.0 - compass_yaw
        # 归一化到 (-π, π]，防止车朝南时 π/2-(-π)=3π/2 超出范围导致 EKF 跳变
        self.g90_enu_yaw = math.atan2(math.sin(enu_yaw), math.cos(enu_yaw))
        # 用接收时刻（ROS 时钟），避免与消息头时间戳的时钟域混用
        self.g90_stamp_ns = self.get_clock().now().nanoseconds

    # === IMU 回调（NED→ENU，200Hz）===

    def _imu_cb(self, msg: Imu):
        now_ns = self.get_clock().now().nanoseconds

        # 判断 G90 航向是否可用（< 2s）
        use_g90 = (self.g90_enu_yaw is not None
                   and (now_ns - self.g90_stamp_ns) < self.G90_TIMEOUT_NS)

        if use_g90:
            enu_yaw = self.g90_enu_yaw
        else:
            # Fallback: H30 磁力计 NED→ENU
            self.get_logger().warn(
                'G90 heading timeout, falling back to H30 magnetometer',
                throttle_duration_sec=5.0)
            q = msg.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            raw_yaw = math.atan2(siny_cosp, cosy_cosp)
            enu_yaw = -raw_yaw + self.yaw_offset
            # 归一化到 (-π, π]
            enu_yaw = math.atan2(math.sin(enu_yaw), math.cos(enu_yaw))

        # 纯 yaw 四元数（two_d_mode=true 只融合 yaw）
        half = enu_yaw * 0.5
        msg.orientation.w = math.cos(half)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(half)

        # orientation_covariance: 让 EKF 正确权衡航向精度
        # [row-major 3x3: roll, pitch, yaw]
        msg.orientation_covariance[0] = 99999.0   # roll (unused, two_d_mode)
        msg.orientation_covariance[4] = 99999.0   # pitch (unused, two_d_mode)
        if use_g90:
            msg.orientation_covariance[8] = 0.000027  # G90 ±0.3° → EKF 强信任
        else:
            msg.orientation_covariance[8] = 0.00122   # H30 ±2° → EKF 弱信任

        # 角速度 FRD→FLU（gx 不变，gy/gz 取反）
        msg.angular_velocity.y = -msg.angular_velocity.y
        msg.angular_velocity.z = -msg.angular_velocity.z

        # 线加速度 FRD→FLU（ax 不变，ay/az 取反）
        msg.linear_acceleration.y = -msg.linear_acceleration.y
        msg.linear_acceleration.z = -msg.linear_acceleration.z

        # 发布
        msg.header.frame_id = 'base_link'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNedToEnu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
