#!/usr/bin/env python3

import cv2
import rclpy
from pathlib import Path
from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


# QoS profile matching the ArduMasterNode
default_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.get_logger().info(f'Initializing DetectionNode...')

        self.bridge = CvBridge()
        model_path = Path.cwd() / "yolo11n-seg.pt"
        self.model = YOLO(model_path)
        self.model.classes = [0]  # person only

        self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        # Internal state flags
        self.guided_mode_active = False
        self.movement_completed = True  # allow first command

        # Subscriber: guided‑mode activation
        self.create_subscription(Bool, '/guided_mode_activation',
                                 self.guided_mode_callback, default_qos)

        # Subscriber: movement completion flag
        self.create_subscription(Bool, '/movement_completion',
                                 self.movement_complete_callback, default_qos)

        # Publisher: send movement commands
        self.movement_pub = self.create_publisher(String, '/movement', default_qos)

        # max yaw deflection, same parameter
        self.declare_parameter("max_angle_rotation", 15.0)
        self.max_angle_rotation = self.get_parameter("max_angle_rotation").value

        self.get_logger().info("DetectionNode started123")

    def guided_mode_callback(self, msg: Bool):
        self.guided_mode_active = msg.data
        self.get_logger().info(f'Guided mode: {self.guided_mode_active}')

    def movement_complete_callback(self, msg: Bool):
        self.movement_completed = msg.data
        self.get_logger().info(f'Movement done: {self.movement_completed}')

    def image_callback(self, msg):

        # Only send commands when in guided mode and last movement is done
        if not self.guided_mode_active or not self.movement_completed:
            return

        try:

            self.movement_completed = False
            response = String()

            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            height, width, _ = frame.shape
            results = self.model(frame, verbose=False)

            if len(results):
                results = results[0]

                # 1) filter for person detections with conf > 0.8
                person_indices = [
                    i for i, cls in enumerate(results.boxes.cls)
                    if int(cls.item()) == 0 and results.boxes.conf[i].item() > 0.80
                ]

                detected = bool(person_indices)
            else:
                detected = False

            if detected:

                # 2) compute mask‐area for each and pick the largest
                areas = []
                for i in person_indices:
                    raw_mask = results.masks.data[i].cpu().numpy()  # model‐mask res
                    mask_rs  = cv2.resize(raw_mask,
                                        (width, height),
                                        interpolation=cv2.INTER_NEAREST).astype(bool)
                    areas.append((i, mask_rs.sum()))
                idx, max_area = max(areas, key=lambda x: x[1])

                x_center = results.boxes.xywh[idx][0].item()
                score = ((x_center / width) - 0.5) * 2
                yaw_angle = -score * self.max_angle_rotation
                yaw_angle = 0 # yaw_angle if abs(yaw_angle) > 10 else 0.0
                area_percentage = (max_area/(width * height)) * 100
                linear_distance = 1.0 if area_percentage < 20 else 0.0
                linear_distance = max(1 - (area_percentage / 20), 0)
                response.data = f'linear_{linear_distance}_yaw_{yaw_angle}'
                self.get_logger().info(f"Person detected with score : {results.boxes.conf[0].item()}, LR score : {score} and yaw_angle : {yaw_angle}, area : {area_percentage}")
            else:
                # no detection → e.g. rotate in place
                yaw = -15
                response.data = f'linear_0.0_yaw_{yaw}'
                self.get_logger().info('Not detected: publishing rotate command')

            self.movement_pub.publish(response)

        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")


def main():
    rclpy.init()
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()