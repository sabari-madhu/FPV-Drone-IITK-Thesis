#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff

COPTER_MODE_GUIDED = 4
TAKEOFF_ALT = 10.5

class CopterTakeoff(Node):
    def __init__(self):
        super().__init__("copter_takeoff")

        self._client_arm = self.create_client(ArmMotors, "/ap/arm_motors")
        self._client_mode_switch = self.create_client(ModeSwitch, "/ap/mode_switch")
        self._client_takeoff = self.create_client(Takeoff, "/ap/experimental/takeoff")

        self._wait_for_services()

        self._cur_geopose = GeoPoseStamped()
        self._cur_localpose = PoseStamped()
        self.curr_altitude = -1

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.create_subscription(GeoPoseStamped, "/ap/geopose/filtered", self.geopose_cb, qos)
        self.create_subscription(PoseStamped, "/ap/pose/filtered", self.localpose_cb, qos)

    def _wait_for_services(self):
        while not self._client_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arm service...')
        while not self._client_mode_switch.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mode switch service...')
        while not self._client_takeoff.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for takeoff service...')

    def geopose_cb(self, msg):
        if msg.header.stamp.sec:
            self._cur_geopose = msg
            self.get_logger().info(f"GeoPose Alt: {msg.pose.position.altitude:.2f}")

    def localpose_cb(self, msg):
        if msg.header.stamp.sec:
            self._cur_localpose = msg.pose.position
            self.curr_altitude = self._cur_localpose.z
            self.get_logger().info(f"Local Z Alt: {self.curr_altitude:.2f}")

    def arm(self):
        req = ArmMotors.Request()
        req.arm = True
        future = self._client_arm.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().result

    def arm_with_timeout(self, timeout_sec):
        start = self.get_clock().now()
        while self.get_clock().now() - start < rclpy.duration.Duration(seconds=timeout_sec):
            if self.arm():
                return True
            time.sleep(1)
        return False

    def switch_mode(self, mode):
        req = ModeSwitch.Request()
        req.mode = mode
        future = self._client_mode_switch.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def switch_mode_with_timeout(self, desired_mode, timeout_sec):
        start = self.get_clock().now()
        while self.get_clock().now() - start < rclpy.duration.Duration(seconds=timeout_sec):
            result = self.switch_mode(desired_mode)
            if result.status or result.curr_mode == desired_mode:
                return True
            time.sleep(1)
        return False

    def takeoff(self, alt):
        req = Takeoff.Request()
        req.alt = alt
        future = self._client_takeoff.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().status

    def takeoff_with_timeout(self, alt, timeout_sec):
        start = self.get_clock().now()
        while self.get_clock().now() - start < rclpy.duration.Duration(seconds=timeout_sec):
            if self.takeoff(alt):
                return True
            time.sleep(1)
        return False

    def get_cur_geopose(self):
        return self._cur_geopose

def main(args=None):
    rclpy.init(args=args)
    node = CopterTakeoff()

    try:
        if not node.switch_mode_with_timeout(COPTER_MODE_GUIDED, 20):
            raise RuntimeError("Failed to switch to GUIDED mode")
        if not node.arm_with_timeout(30):
            raise RuntimeError("Failed to arm")
        if not node.takeoff_with_timeout(TAKEOFF_ALT, 20):
            raise RuntimeError("Takeoff command failed")

        while node.get_cur_geopose().pose.position.altitude < TAKEOFF_ALT:
            rclpy.spin_once(node)
            time.sleep(1)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
