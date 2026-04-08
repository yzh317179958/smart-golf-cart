"""S11 深度话题修复节点

修复 S11 两个非标准问题，让 depthimage_to_laserscan 能正常工作：
1. CameraInfo distortion_model: LX_DISTORTION_FISHEYE → plumb_bob
2. CameraInfo P 矩阵: 非标准 → 标准 [fx 0 cx 0; 0 fy cy 0; 0 0 1 0]
3. Depth Image encoding: mono16 → 16UC1（内容完全相同，只改 encoding 字符串）
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CameraInfoFix(Node):

    def __init__(self):
        super().__init__('camera_info_fix')
        self.declare_parameter('input_topic', '/lx_camera_node/LxCamera_TofInfo')
        self.declare_parameter('output_topic', '/depth_camera_info_fixed')
        self.declare_parameter('depth_input', '/lx_camera_node/LxCamera_Depth')
        self.declare_parameter('depth_output', '/depth_image_fixed')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        depth_input = self.get_parameter('depth_input').value
        depth_output = self.get_parameter('depth_output').value

        self.info_pub = self.create_publisher(CameraInfo, output_topic, 10)
        self.depth_pub = self.create_publisher(Image, depth_output, 10)
        self.create_subscription(CameraInfo, input_topic, self.info_cb, 10)
        self.create_subscription(Image, depth_input, self.depth_cb, 10)

    def info_cb(self, msg: CameraInfo):
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx = msg.k[0]
        fy = msg.k[4]
        cx = msg.k[2]
        cy = msg.k[5]
        msg.p = [fx, 0.0, cx, 0.0,
                 0.0, fy, cy, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(msg)

    def depth_cb(self, msg: Image):
        if msg.encoding == 'mono16':
            msg.encoding = '16UC1'
        self.depth_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
