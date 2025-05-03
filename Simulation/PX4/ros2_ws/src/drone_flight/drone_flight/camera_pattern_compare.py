import cv2
import rclpy
import numpy as np
from pathlib import Path
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class PatternDetector:
    def __init__(self, pattern_image, logger):
        self.logger = logger
        self.pattern = pattern_image
        self.logger.info(f"CV version : {cv2.__version__}")
        self.pattern_gray = cv2.cvtColor(pattern_image, cv2.COLOR_BGR2GRAY)
        self.sift = cv2.SIFT_create()
        self.pattern_kp, self.pattern_des = self.sift.detectAndCompute(self.pattern_gray, None)

    def detect_blue_regions(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        kernel = np.ones((3, 3), np.uint8)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
        return blue_mask

    def find_feature_matches(self, frame_gray):
        frame_kp, frame_des = self.sift.detectAndCompute(frame_gray, None)
        if frame_des is None or len(frame_des) < 2:
            return []
        bf = cv2.BFMatcher()
        try:
            matches = bf.knnMatch(self.pattern_des, frame_des, k=2)
        except:
            return []
        good_matches = [m for m, n in matches if m.distance < 0.75 * n.distance]
        return good_matches

    def multi_scale_template_matching(self, frame_gray):
        template_matches = []
        frame_height, frame_width = frame_gray.shape
        pattern_height, pattern_width = self.pattern_gray.shape

        initial_scale = 1.0
        if pattern_height > frame_height or pattern_width > frame_width:
            scale_height = frame_height / pattern_height * 0.9
            scale_width = frame_width / pattern_width * 0.9
            initial_scale = min(scale_height, scale_width)

        scales = np.linspace(initial_scale, 0.1, 20)
        for scale in scales:
            width = int(self.pattern_gray.shape[1] * scale)
            height = int(self.pattern_gray.shape[0] * scale)
            if width < 10 or height < 10 or width >= frame_width or height >= frame_height:
                continue
            resized_pattern = cv2.resize(self.pattern_gray, (width, height))
            try:
                result = cv2.matchTemplate(frame_gray, resized_pattern, cv2.TM_CCOEFF_NORMED)
                threshold = 0.6
                locations = np.where(result >= threshold)
                for pt in zip(*locations[::-1]):
                    template_matches.append({
                        'position': pt,
                        'size': (width, height),
                        'score': result[pt[1], pt[0]]
                    })
            except cv2.error:
                continue
        return template_matches

    def detect_pattern(self, frame):
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blue_mask = self.detect_blue_regions(frame)
        feature_matches = self.find_feature_matches(frame_gray)
        template_matches = self.multi_scale_template_matching(frame_gray)
        confidence = self.calculate_confidence(blue_mask, feature_matches, template_matches, frame)
        return confidence, blue_mask, len(feature_matches), len(template_matches)

    def calculate_confidence(self, blue_mask, feature_matches, template_matches, frame):
        blue_pixels = np.sum(blue_mask > 0)
        blue_score = min(1.0, blue_pixels / (frame.shape[0] * frame.shape[1] * 0.1))
        feature_score = min(1.0, len(feature_matches) / 20.0)
        template_score = min(1.0, len(template_matches) / 5.0)
        weights = [0.4, 0.3, 0.3]
        final_score = (
            weights[0] * blue_score +
            weights[1] * feature_score +
            weights[2] * template_score
        )
        return final_score


def normalize_value(x):
    if x < 0 or x > 1:
        raise ValueError("Input must be in [0, 1]")
    if x < 0.1:
        return (x / 0.1) * 0.7
    elif x <= 0.5:
        return 0.7 + ((x - 0.1) / 0.4) * 0.2
    else:
        return 0.9 + ((x - 0.5) / 0.5) * 0.1


class PatternDetectionNode(Node):
    def __init__(self):
        super().__init__('pattern_detection_node')
        self.bridge = CvBridge()
        self.pattern_image = cv2.imread(str(Path.cwd() / "pattern.png"))
        if self.pattern_image is None:
            self.get_logger().error('Failed to load pattern image!')
            return

        self.detector = PatternDetector(self.pattern_image, self.get_logger())

        gst_pipeline = (
            "udpsrc port=5600 caps=application/x-rtp ! "
            "rtph264depay ! "
            "h264parse ! "
            "avdec_h264 ! "
            "videoconvert ! "
            "appsink"
        )
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera!')
            return

        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.score_publisher = self.create_publisher(Float32, '/pattern_score', qos_profile)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.use_timer = True
        self.get_logger().info('Pattern Detection Node initialized')

    def timer_callback(self):
        if not self.use_timer:
            return
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return
        confidence, blue_mask, num_features, num_templates = self.detector.detect_pattern(frame)
        score_msg = Float32()
        score_msg.data = float(normalize_value(confidence))
        self.get_logger().warn(f'Score is {score_msg.data}')
        self.score_publisher.publish(score_msg)

    def camera_callback(self, msg):
        if self.use_timer:
            return
        try:
            current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            score = self.calculate_pattern_significance(current_frame, self.pattern_image)
            score_msg = Float32()
            score_msg.data = float(score)
            self.score_publisher.publish(score_msg)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def calculate_pattern_significance(self, frame, pattern_image):
        if pattern_image is None:
            self.get_logger().warn("Pattern image not available. Returning a score of 0.")
            return 0.0
        frame_height, frame_width = frame.shape[:2]
        pattern_height, pattern_width = pattern_image.shape[:2]
        if pattern_height > frame_height or pattern_width > frame_width:
            scale_factor = min(frame_height / pattern_height, frame_width / pattern_width)
            pattern_image = cv2.resize(pattern_image, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_AREA)
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        pattern_gray = cv2.cvtColor(pattern_image, cv2.COLOR_BGR2GRAY)
        result = cv2.matchTemplate(frame_gray, pattern_gray, cv2.TM_CCOEFF_NORMED)
        threshold = 0.8
        match_locations = np.where(result >= threshold)
        match_count = len(match_locations[0])
        pattern_area = pattern_image.shape[0] * pattern_image.shape[1]
        frame_area = frame.shape[0] * frame.shape[1]
        if match_count > 0:
            total_pattern_area = match_count * pattern_area
            coverage_percentage = total_pattern_area / frame_area
            significance_score = min(coverage_percentage, 1.0)
        else:
            significance_score = 0.0
        return significance_score

    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    print('Starting camera pattern matching node...')
    rclpy.init(args=args)
    node = PatternDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
