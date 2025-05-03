#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String

class Tracking(Node):
    def __init__(self):
        super().__init__('tracking')
        self.detection_topic = "/internal/person_detection"
        self.cmd_topic = "/internal/cmd_vel_tracking"
        self.create_subscription(String, self.detection_topic, self.detection_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        
        # Hardcoded tracking gains and target scale (desired bounding box size)
        self.target_scale = 0.4   # Desired relative width of the person (proxy for distance)
        self.k_forward = 1.0      # Forward velocity gain
        self.k_turn = 1.0         # Angular velocity gain
        self.search_turn_speed = 0.3  # When no detection
        
        self.last_detection = None
        self.get_logger().info("Tracking node started.")
        
    def detection_callback(self, msg: String):
        self.last_detection = msg.data
        self.get_logger().info(f"Received detection: {self.last_detection}")
        self.publish_command()
        
    def _publish_command(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "base_link"
        
        # If no detection, perform a search (rotate slowly)
        if self.last_detection is None or self.last_detection == "none":
            twist.twist.linear.x = 0.0
            twist.twist.angular.z = self.search_turn_speed
        else:
            try:
                parts = self.last_detection.split("_")
                if len(parts) != 3:
                    self.get_logger().warn("Invalid detection format, searching...")
                    twist.twist.linear.x = 0.0
                    twist.twist.angular.z = self.search_turn_speed
                else:
                    # Parse detection result: region_offset_scale (e.g., "center_0.00_0.35")
                    region, offset_str, scale_str = parts
                    offset = float(offset_str)
                    scale = float(scale_str)
                    # Angular velocity proportional to offset (turn to center the person)
                    angular_velocity = self.k_turn * offset
                    # Forward velocity proportional to how far the person is (smaller scale means farther)
                    error_scale = self.target_scale - scale
                    forward_velocity = self.k_forward * error_scale if error_scale > 0 else 0.0
                    twist.twist.linear.x = forward_velocity
                    twist.twist.angular.z = angular_velocity
            except Exception as e:
                self.get_logger().error(f"Error parsing detection: {e}")
                twist.twist.linear.x = 0.0
                twist.twist.angular.z = self.search_turn_speed
        
        self.cmd_pub.publish(twist)
        self.get_logger().info(f"Published command: linear.x={twist.twist.linear.x}, angular.z={twist.twist.angular.z}")

    def publish_command(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "base_link"

        offset_threshold = 0.1  # Tweak this based on testing

        if self.last_detection is None or self.last_detection == "none":
            twist.twist.linear.x = 0.0
            twist.twist.angular.z = self.search_turn_speed
        else:
            try:
                parts = self.last_detection.split("_")
                if len(parts) != 3:
                    self.get_logger().warn("Invalid detection format, searching...")
                    twist.twist.linear.x = 0.0
                    twist.twist.angular.z = self.search_turn_speed
                else:
                    region, offset_str, scale_str = parts
                    offset = float(offset_str)
                    scale = float(scale_str)

                    if abs(offset) > offset_threshold:
                        # Object is not centered; rotate drone
                        twist.twist.linear.x = 0.0
                        twist.twist.angular.z = self.k_turn * offset
                    else:
                        # Object centered; move forward
                        error_scale = self.target_scale - scale
                        twist.twist.linear.x = self.k_forward * error_scale if error_scale > 0 else 0.0
                        twist.twist.angular.z = 0.0  # Optionally maintain slight yaw adjustment here
            except Exception as e:
                self.get_logger().error(f"Error parsing detection: {e}")
                twist.twist.linear.x = 0.0
                twist.twist.angular.z = self.search_turn_speed

        self.cmd_pub.publish(twist)
        self.get_logger().info(f"Published command: linear.x={twist.twist.linear.x}, angular.z={twist.twist.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = Tracking()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Tracking node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
