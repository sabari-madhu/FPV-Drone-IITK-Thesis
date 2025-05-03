#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import urllib.request
import sys
import os

def setup_face_detection():
    # Get the path to the cascade file
    # Method 1: Direct path to the XML file
    cascade_path = 'haarcascade_frontalface_default.xml'
    
    # Method 2: If you want to download it if not present
    if not os.path.exists(cascade_path):
        
        url = "https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml"
        urllib.request.urlretrieve(url, cascade_path)
    
    # Load the cascade classifier
    face_cascade = cv2.CascadeClassifier(cascade_path)
    
    if face_cascade.empty():
        raise RuntimeError("Error: Could not load face cascade classifier")
    
    return face_cascade

def setup_profile_detection():
    # Get the path to the cascade file
    # Method 1: Direct path to the XML file
    cascade_path = 'haarcascade_profileface.xml'
    
    # Method 2: If you want to download it if not present
    if not os.path.exists(cascade_path):
        url = "https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_profileface.xml"
        urllib.request.urlretrieve(url, cascade_path)
    
    # Load the cascade classifier
    profile_cascade = cv2.CascadeClassifier(cascade_path)
    
    if profile_cascade.empty():
        raise RuntimeError("Error: Could not load face cascade classifier")
    
    return profile_cascade

class HumanDetectionNode(Node):
    def __init__(self):
        super().__init__('human_detection_node')
        
        # Create subscription to the input image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10)
        
        # Create publisher for the detection results
        self.publisher = self.create_publisher(
            String,
            '/human_detection/results',
            10)
        
        # Initialize CV bridge
        self.bridge = CvBridge()

        self.get_logger().info(f"The version of cv2 is : {cv2.__version__}")
        
        # Initialize face cascades
        # self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        # self.profile_cascade = cv2.CascadeClassifier('haarcascade_profileface.xml')
        self.face_cascade = setup_face_detection()
        self.profile_cascade = setup_profile_detection()
        
        self.get_logger().info('Human Detection Node has been started')

        self.get_logger().info(f"The path of python is : {str(sys.executable)}")

    def process_per_frame_fisheye(self, frame, width, height):
        """
        Correct fisheye distortion in a frame using given calibration parameters.
        """
        # Normalized intrinsic parameters
        cx, cy = 0.5, 0.5  # Center coordinates (normalized)
        k1, k2 = -0.230, -0.020  # Radial distortion coefficients

        # Camera intrinsic matrix
        cx_px = cx * width
        cy_px = cy * height
        K = np.array([
            [width, 0, cx_px],
            [0, height, cy_px],
            [0, 0, 1]
        ], dtype=np.float32)

        # Distortion coefficients
        D = np.array([k1, k2, 0.09, 0], dtype=np.float32)

        # Undistortion map for fisheye
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            K, D, np.eye(3), K, (width, height), cv2.CV_16SC2
        )

        # Apply remapping
        corrected_frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)
        return corrected_frame

    def split_image_into_thirds(self, img):
        """Split the image into three vertical sections."""
        height, width = img.shape[:2]
        section_width = width // 3
        
        left_section = img[:, :section_width]
        middle_section = img[:, section_width:section_width*2]
        right_section = img[:, section_width*2:]
        
        return left_section, middle_section, right_section

    def detect_head(self, img):
        """Detect human heads in an image section."""
        try:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Detect faces (both frontal and profile)
            frontal_faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
            profile_faces = self.profile_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
            
            # Combine detections
            all_detections = list(frontal_faces) + list(profile_faces)
            
            return len(all_detections) > 0
            
        except Exception as e:
            self.get_logger().error(f'Error in head detection: {str(e)}')
            return False

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Get image dimensions
            height, width = cv_image.shape[:2]
            
            # Remove fisheye distortion
            corrected_image = self.process_per_frame_fisheye(cv_image, width, height)
            
            # Split image into thirds
            left, middle, right = self.split_image_into_thirds(corrected_image)
            
            # Detect humans in each section
            left_detection = 1 if self.detect_head(left) else 0
            middle_detection = 1 if self.detect_head(middle) else 0
            right_detection = 1 if self.detect_head(right) else 0
            
            # Create result string
            result = f"{left_detection},{middle_detection},{right_detection}"
            self.get_logger().info(f"The result is : {result}")
            # Publish results
            msg = String()
            msg.data = result
            self.publisher.publish(msg)
            
            # Log results for debugging
            self.get_logger().debug(f'Published detection results: {result}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    human_detector = HumanDetectionNode()
    
    try:
        rclpy.spin(human_detector)
    except KeyboardInterrupt:
        pass
    finally:
        human_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()