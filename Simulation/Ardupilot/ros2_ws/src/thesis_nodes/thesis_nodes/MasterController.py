#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from geographic_msgs.msg import GeoPoseStamped
from std_msgs.msg import String

class MasterDrone(Node):
    def __init__(self):
        super().__init__('master_drone')
        # Parameter: desired altitude for takeoff
        self.takeoff_altitude = 10.0  # meters (adjust as needed)

        # Internal state variables
        self.altitude_reached = False
        self.current_detection = "none"
        self.last_scan_cmd = None
        self.last_tracking_cmd = None

        # Subscribers
        self.create_subscription(GeoPoseStamped, '/internal/geopose', self.geopose_callback, 10)
        self.create_subscription(String, '/internal/person_detection', self.detection_callback, 10)
        self.create_subscription(TwistStamped, '/internal/cmd_vel_scan', self.scan_cmd_callback, 10)
        self.create_subscription(TwistStamped, '/internal/cmd_vel_tracking', self.tracking_cmd_callback, 10)

        # Publisher: This is the final command forwarded to ArdupilotCommunicator
        self.cmd_pub = self.create_publisher(TwistStamped, '/internal/cmd_vel', 10)

        # Timer for periodically selecting and forwarding the command
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("MasterDrone node started.")

    def geopose_callback(self, msg: GeoPoseStamped):
        self.get_logger().debug(f"Geopose message : {msg}")
        altitude = msg.pose.position.altitude
        if altitude >= self.takeoff_altitude:
            if not self.altitude_reached:
                self.get_logger().info(f"Takeoff altitude reached: {altitude:.2f} m")
            self.altitude_reached = True
        else:
            self.altitude_reached = False
            self.get_logger().info(f"Altitude ({altitude:.2f} m) below threshold ({self.takeoff_altitude:.2f} m)")

    def detection_callback(self, msg: String):
        self.current_detection = msg.data
        self.get_logger().info(f"Detection updated: {self.current_detection}")

    def scan_cmd_callback(self, msg: TwistStamped):
        self.last_scan_cmd = msg
        self.get_logger().debug(f"Received scan command : {msg}")

    def tracking_cmd_callback(self, msg: TwistStamped):
        self.last_tracking_cmd = msg
        self.get_logger().debug("Received tracking command")

    def timer_callback(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"

        if not self.altitude_reached:
            # Still waiting for the drone to reach takeoff altitude.
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.get_logger().info("Waiting for takeoff altitude.")
        else:
            # Once altitude is reached, decide between tracking and scan.
            if self.current_detection != "none":
                # A person is detected, so use the tracking command if available.
                if self.last_tracking_cmd is not None:
                    cmd = self.last_tracking_cmd
                    cmd.header.stamp = self.get_clock().now().to_msg()
                    self.get_logger().debug("Using tracking command.")
                else:
                    self.get_logger().warn("Tracking command not available; holding position.")
                    cmd.twist.linear.x = 0.0
                    cmd.twist.angular.z = 0.0
            else:
                # No person detected, so use the scan command if available.
                if self.last_scan_cmd is not None:
                    cmd = self.last_scan_cmd
                    cmd.header.stamp = self.get_clock().now().to_msg()
                    self.get_logger().info("Using scan command.")
                else:
                    self.get_logger().warn("Scan command not available; holding position.")
                    cmd.twist.linear.x = 0.0
                    cmd.twist.angular.z = 0.0

        # Publish the selected command
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MasterDrone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MasterDrone node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
