#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloPersonSegNode(Node):
    def __init__(self):
        super().__init__('yolo_person_seg_node')
        self.bridge = CvBridge()

        self.model = YOLO("yolo11n-seg.pt")
        self.model.classes = [0]

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.image_pub = self.create_publisher(
            Image,
            '/annotated_images',
            10
        )

        self.score_pub = self.create_publisher(
            Float32,
            '/person_horizontal_score',
            10
        )

        self.get_logger().info("YOLO Person Segmentation Node Started")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = frame.shape
            results = self.model(frame)[0]

            person_indices = [
                i for i, cls in enumerate(results.boxes.cls)
                if int(cls.item()) == 0 and results.boxes.conf[i].item() > 0.75
            ]

            if not person_indices:
                return

            idx = person_indices[0]
            box = results.boxes.xywh[idx]
            x_center = box[0].item()
            score = ((x_center / width) - 0.5) * 2

            score_msg = Float32()
            score_msg.data = float(score)
            self.score_pub.publish(score_msg)

            results.boxes = results.boxes[person_indices]
            if results.masks is not None:
                results.masks.data = results.masks.data[person_indices]

            annotated = results.plot()
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_msg.header = msg.header
            self.image_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to process frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloPersonSegNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
