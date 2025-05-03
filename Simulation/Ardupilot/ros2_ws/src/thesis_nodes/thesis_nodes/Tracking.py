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

        # Parameters
        self.offset_threshold = 0.1
        self.k_turn = 1.0
        self.forward_speed = 0.2  # Fixed slow forward speed
        self.search_turn_speed = 0.3

        self.prev_scale = None  # Track previous detection size
        self.last_detection = None

        self.get_logger().info("Tracking node with improved logic started.")

    def detection_callback(self, msg: String):
        self.last_detection = msg.data
        self.publish_command()

    def publish_command(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "base_link"

        offset_threshold = 0.1  # Adjust experimentally

        if self.last_detection is None or self.last_detection == "none":
            # Searching behavior
            twist.twist.linear.x = 0.0
            twist.twist.angular.z = self.search_turn_speed
            self.prev_scale = None  # Reset previous detection scale
        else:
            try:
                parts = self.last_detection.split("_")
                if len(parts) != 3:
                    self.get_logger().warn("Invalid detection format, entering search mode...")
                    twist.twist.linear.x = 0.0
                    twist.twist.angular.z = self.search_turn_speed
                    self.prev_scale = None
                else:
                    region, offset_str, scale_str = parts
                    offset = float(offset_str)
                    scale = float(scale_str)

                    if abs(offset) > offset_threshold:
                        # Not centered yet: rotate to center
                        twist.twist.linear.x = 0.0
                        twist.twist.angular.z = self.k_turn * offset
                        self.prev_scale = None  # Reset scale since we're re-centering
                    else:
                        # Centered: decide forward movement based on bounding box scale change
                        if self.prev_scale is None or scale > self.prev_scale:
                            twist.twist.linear.x = self.forward_speed
                            twist.twist.angular.z = 0.0
                        else:
                            twist.twist.linear.x = 0.0
                            twist.twist.angular.z = 0.0
                        self.prev_scale = scale  # Update scale for next comparison
            except Exception as e:
                self.get_logger().error(f"Error processing detection: {e}")
                twist.twist.linear.x = 0.0
                twist.twist.angular.z = self.search_turn_speed
                self.prev_scale = None

        try:
            self.cmd_pub.publish(twist)
            self.get_logger().info(
                f"Published command: linear.x={twist.twist.linear.x}, angular.z={twist.twist.angular.z}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to publish command: {e}")

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
