"""调试图像缩放节点：将 dbg_image 缩小为 320x270 发布到 /dbg_small"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageResizer(Node):

    def __init__(self):
        super().__init__('image_resizer')
        self.pub = self.create_publisher(Image, '/dbg_small', 5)
        self.create_subscription(Image, '/yolo/dbg_image', self.cb, 5)
        self.get_logger().info('图像缩放节点启动 → /dbg_small (320x270)')

    def cb(self, msg: Image):
        # bgr8 or rgb8
        if msg.encoding in ('bgr8', 'rgb8'):
            dtype, channels = np.uint8, 3
        else:
            return

        img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)
        small = cv2.resize(img, (320, 270), interpolation=cv2.INTER_AREA)

        out = Image()
        out.header = msg.header
        out.height, out.width = 270, 320
        out.encoding = msg.encoding
        out.step = 320 * channels
        out.data = small.tobytes()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImageResizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
