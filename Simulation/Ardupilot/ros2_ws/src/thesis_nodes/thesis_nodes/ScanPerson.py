#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class Scan(Node):
    def __init__(self):
        super().__init__('scan')
        self._cmd_pub = self.create_publisher(TwistStamped, '/internal/cmd_vel_scan', 10)
        # Hardcoded motion parameters
        self.linear_speed = 1.0   # m/s forward
        self.angular_speed = 0.5  # rad/s yaw rate
        timer_period = 0.1        # publish at 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Scan node started; publishing circular movement commands.")

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = self.linear_speed
        msg.twist.angular.z = self.angular_speed
        self._cmd_pub.publish(msg)
        self.get_logger().debug("Published circular movement command.")

def main(args=None):
    rclpy.init(args=args)
    node = Scan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Scan node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
