#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from std_msgs.msg import Bool
import time
import sys

class MavlinkTakeoffNode(Node):
    def __init__(self):
        super().__init__('mavlink_takeoff_node')
        self.declare_parameter('mavlink_connection', '/dev/serial/by-id/usb-CubePilot_CubeOrange+_2A002F000E51323138363132-if00')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('takeoff_altitude', 10.0)
        self.completion_pub = self.create_publisher(Bool, 'takeoff_complete', 10)
        self.get_logger().info('Starting takeoff sequence')
        self.takeoff_timer = self.create_timer(0.1, self.execute_takeoff)

    def execute_takeoff(self):
        self.takeoff_timer.cancel()
        try:
            connection = mavutil.mavlink_connection(
                self.get_parameter('mavlink_connection').value,
                baud=self.get_parameter('baud_rate').value
            )

            self.get_logger().info("Waiting for heartbeat...")
            connection.wait_heartbeat()
            self.get_logger().info(f"Heartbeat from system {connection.target_system}, component {connection.target_component}")

            connection.mav.rc_channels_override_send(
                connection.target_system,
                connection.target_component,
                65535, 65535, 1000, 65535,
                65535, 65535, 65535, 65535
            )
            self.get_logger().info("Set RC channel 3 (throttle) to 1000")
            time.sleep(2)

            connection.mav.set_mode_send(
                connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4
            )

            mode_changed = False
            for _ in range(30):
                msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=0.1)
                if msg and msg.custom_mode == 4:
                    mode_changed = True
                    break

            if not mode_changed:
                self.get_logger().error("Failed to enter GUIDED mode")
                self.publish_completion(False)
                connection.close()
                self.cleanup_and_shutdown()
                return

            self.get_logger().info("GUIDED mode confirmed")

            connection.mav.command_long_send(
                connection.target_system,
                connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )

            ack = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
            if not ack or ack.result != 0:
                self.get_logger().error(f"Arming failed: {ack.result if ack else 'timeout'}")
                self.publish_completion(False)
                connection.close()
                self.cleanup_and_shutdown()
                return

            self.get_logger().info("Armed successfully")
            time.sleep(2)

            takeoff_alt = self.get_parameter('takeoff_altitude').value
            connection.mav.command_long_send(
                connection.target_system,
                connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, takeoff_alt
            )

            takeoff_ack = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
            if not takeoff_ack or takeoff_ack.result != 0:
                self.get_logger().error(f"Takeoff command failed: {takeoff_ack.result if takeoff_ack else 'timeout'}")
                self.publish_completion(False)
                connection.close()
                self.cleanup_and_shutdown()
                return

            self.get_logger().info("Takeoff command accepted")

            while True:
                msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
                if msg:
                    alt = msg.relative_alt / 1000
                    self.get_logger().info(f"Current altitude: {alt:.2f}m")
                    if alt >= takeoff_alt * 0.95:
                        self.get_logger().info("Reached target altitude")
                        connection.mav.rc_channels_override_send(
                            connection.target_system,
                            connection.target_component,
                            65535, 65535, 65535, 1500,
                            65535, 65535, 65535, 65535
                        )
                        self.get_logger().info("Centered yaw (RC channel 4)")
                        self.publish_completion(True)
                        connection.close()
                        self.cleanup_and_shutdown()
                        break
                else:
                    self.get_logger().error("Altitude message timeout")
                    self.publish_completion(False)
                    connection.close()
                    self.cleanup_and_shutdown()
                    break

        except Exception as e:
            self.get_logger().error(f"Takeoff error: {str(e)}")
            self.publish_completion(False)
            self.cleanup_and_shutdown()

    def publish_completion(self, success):
        msg = Bool()
        msg.data = success
        self.completion_pub.publish(msg)

    def cleanup_and_shutdown(self):
        self.destroy_node()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = MavlinkTakeoffNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
