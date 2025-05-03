#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO

class CameraModule(Node):
    def __init__(self):
        super().__init__('camera_module')
        # Hardcoded topics and model path
        self.camera_topic = "/camera/image"
        self.detection_topic = "/internal/person_detection"
        self.model_path = "yolo11n.pt"
        
        self.get_logger().info(f"Loading YOLO model from {self.model_path}")
        self.yolo_model = YOLO(self.model_path)
        
        self.detection_pub = self.create_publisher(String, self.detection_topic, 10)
        self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
        self.bridge = CvBridge()
        
        self.get_logger().info("CameraModule node started.")
    
    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error("Image conversion failed: " + str(e))
            return
        
        height, width, _ = cv_image.shape
        
        # Run YOLO detection on the entire image
        results = self.yolo_model.predict(source=[cv_image], verbose=False)
        result = results[0]
        
        person_detections = []
        for box in result.boxes:
            cls_id = int(box.cls[0].item()) if len(box.cls) > 0 else None
            # if cls_id is not None and self.yolo_model.names[cls_id] == "person":
            person_detections.append(box)
        
        detection_msg = String()
        if not person_detections:
            detection_msg.data = "none"
        else:
            # Select the best detection (highest confidence)
            best_box = max(person_detections, key=lambda b: b.conf[0].item())
            x1, y1, x2, y2 = map(int, best_box.xyxy[0])
            box_center_x = (x1 + x2) / 2.0
            # Normalized offset: -1 (far left) to +1 (far right)
            offset = (box_center_x - (width/2)) / (width/2)
            # Region based on thirds of the image width
            if box_center_x < width/3:
                region = "left"
            elif box_center_x < 2*width/3:
                region = "center"
            else:
                region = "right"
            bbox_width = x2 - x1
            scale = bbox_width / width
            detection_msg.data = f"{region}_{offset:.2f}_{scale:.2f}"
        
        self.detection_pub.publish(detection_msg)
        self.get_logger().info(f"Published detection: {detection_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraModule()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down CameraModule node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
