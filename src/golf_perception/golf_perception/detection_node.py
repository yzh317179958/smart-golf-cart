"""YOLOv8-Pose 行人检测 ROS2 节点

订阅 S11 RGB 图像，使用 YOLOv8s-Pose TensorRT 推理，
发布检测结果（bbox + 17个 COCO 关键点 + 置信度）。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2DArray, Detection2D, ObjectHypothesisWithPose,
    BoundingBox2D,
)
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Header
from cv_bridge import CvBridge

import numpy as np


class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')

        # 参数
        self.declare_parameter('model_path', 'yolov8s-pose.engine')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_topic', '/lx_camera_node/LxCamera_Rgb')
        self.declare_parameter('device', 0)

        model_path = self.get_parameter('model_path').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        image_topic = self.get_parameter('image_topic').value
        device = self.get_parameter('device').value

        # 加载 YOLOv8-Pose 模型
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            self.get_logger().info(f'模型加载成功: {model_path}')
        except Exception as e:
            self.get_logger().error(f'模型加载失败: {e}')
            raise

        self.bridge = CvBridge()

        # 订阅 S11 RGB 图像
        self.sub_image = self.create_subscription(
            Image, image_topic, self.image_callback, 5)

        # 发布检测结果（标准 vision_msgs）
        self.pub_detections = self.create_publisher(
            Detection2DArray, '/detections', 10)

        # 发布关键点（自定义话题，JSON 字符串，供手势/跟踪节点使用）
        from std_msgs.msg import String
        self.pub_keypoints = self.create_publisher(
            String, '/detection_keypoints', 10)

        self.get_logger().info(
            f'检测节点启动 | 话题: {image_topic} | 置信度: {self.conf_thresh}')

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'图像转换失败: {e}')
            return

        # YOLOv8-Pose 推理
        results = self.model.predict(
            frame,
            conf=self.conf_thresh,
            verbose=False,
        )

        det_array = Detection2DArray()
        det_array.header = msg.header

        keypoints_list = []

        if results and len(results) > 0:
            result = results[0]

            if result.boxes is not None:
                boxes = result.boxes.xyxy.cpu().numpy()
                confs = result.boxes.conf.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy()

                # 关键点（17×3: x, y, confidence）
                kpts = None
                if result.keypoints is not None:
                    kpts = result.keypoints.data.cpu().numpy()

                for i in range(len(boxes)):
                    cls_id = int(classes[i])
                    # 只检测行人（COCO class 0 = person）
                    if cls_id != 0:
                        continue

                    x1, y1, x2, y2 = boxes[i]
                    conf = float(confs[i])

                    det = Detection2D()
                    det.header = msg.header

                    # bbox（中心点 + 宽高）
                    det.bbox = BoundingBox2D()
                    det.bbox.center = Pose2D()
                    det.bbox.center.x = float((x1 + x2) / 2)
                    det.bbox.center.y = float((y1 + y2) / 2)
                    det.bbox.size_x = float(x2 - x1)
                    det.bbox.size_y = float(y2 - y1)

                    # 置信度
                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = 'person'
                    hyp.hypothesis.score = conf
                    det.results.append(hyp)

                    det_array.detections.append(det)

                    # 关键点数据（vision_msgs 没有 keypoints 字段，用单独话题发布）
                    if kpts is not None and i < len(kpts):
                        kp = kpts[i].tolist()  # 17×3
                        keypoints_list.append({
                            'bbox': [float(x1), float(y1), float(x2), float(y2)],
                            'confidence': conf,
                            'keypoints': kp,
                        })

        self.pub_detections.publish(det_array)

        # 发布关键点 JSON
        if keypoints_list:
            import json
            from std_msgs.msg import String
            kp_msg = String()
            kp_msg.data = json.dumps(keypoints_list)
            self.pub_keypoints.publish(kp_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
