#!/usr/bin/env python3

import rclpy
import time
import numpy as np
from rclpy.node import Node
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import TwistStamped, PoseStamped
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff

COPTER_MODE_GUIDED = 4
COPTER_MODE_RTL = 6
COPTER_MODE_TAKEOFF = 13

TAKEOFF_ALT = 10.0
FORWARD_SPEED = 2.0
ROTATION_SPEED = 0.5

class CopterTakeoff(Node):
    def __init__(self):
        super().__init__("copter_takeoff")

        self.declare_parameter("arm_topic", "/ap/arm_motors")
        self._client_arm = self.create_client(ArmMotors, self.get_parameter("arm_topic").value)
        while not self._client_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arm service not available')

        self.declare_parameter("mode_topic", "/ap/mode_switch")
        self._client_mode_switch = self.create_client(ModeSwitch, self.get_parameter("mode_topic").value)
        while not self._client_mode_switch.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('mode switch service not available')

        self.declare_parameter("geopose_topic", "/ap/geopose/filtered")
        self._subscription_geopose = self.create_subscription(
            GeoPoseStamped, self.get_parameter("geopose_topic").value,
            self.geopose_cb, rclpy.qos.QoSProfile(depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE)
        )
        self._cur_geopose = GeoPoseStamped()

        self._cmd_vel_pub = self.create_publisher(TwistStamped, '/ap/cmd_vel', 10)

        self.declare_parameter("localpose_topic", "/ap/pose/filtered")
        self._subscription_localpose = self.create_subscription(
            PoseStamped, self.get_parameter("localpose_topic").value,
            self.localpose_cb, rclpy.qos.QoSProfile(depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE)
        )
        self._cur_localpose = PoseStamped()
        self.curr_altitude = -1

    def localpose_cb(self, msg: PoseStamped):
        self._cur_localpose = msg.pose.position
        self.curr_altitude = self._cur_localpose.z

    def geopose_cb(self, msg: GeoPoseStamped):
        self._cur_geopose = msg

    def publish_velocity_command(self, linear_x=0.0, linear_y=0.0, linear_z=0.0,
                                 angular_x=0.0, angular_y=0.0, angular_z=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear_x
        msg.twist.linear.y = linear_y
        msg.twist.linear.z = linear_z
        msg.twist.angular.x = angular_x
        msg.twist.angular.y = angular_y
        msg.twist.angular.z = angular_z
        self._cmd_vel_pub.publish(msg)

    def execute_square_movement(self):
        SIDE_LENGTH = 5
        NUM_SIDES = 4
        RAMP_TIME = 1.0
        ALTITUDE_TARGET = TAKEOFF_ALT

        def get_ramped_speed(current_time, total_time, max_speed):
            if current_time < RAMP_TIME:
                return (current_time / RAMP_TIME) * max_speed
            elif current_time > total_time - RAMP_TIME:
                return ((total_time - current_time) / RAMP_TIME) * max_speed
            return max_speed

        def maintain_altitude():
            current_alt = self.curr_altitude
            alt_error = ALTITUDE_TARGET - current_alt
            return 0.9 * alt_error

        for _ in range(NUM_SIDES):
            forward_time = (SIDE_LENGTH / FORWARD_SPEED) + (2 * RAMP_TIME)
            rotation_time = (np.pi / 2) / ROTATION_SPEED + (2 * RAMP_TIME)

            start_time = time.time()
            while time.time() - start_time < forward_time:
                t = time.time() - start_time
                v = get_ramped_speed(t, forward_time, FORWARD_SPEED)
                vz = maintain_altitude()
                if abs(vz) > 2.0:
                    v *= 0.5
                if abs(vz) > 5.0:
                    self.publish_velocity_command()
                    return
                self.publish_velocity_command(linear_x=v, linear_z=vz)
                rclpy.spin_once(self, timeout_sec=0.1)
            self.publish_velocity_command()
            time.sleep(0.5)

            start_time = time.time()
            while time.time() - start_time < rotation_time:
                t = time.time() - start_time
                w = get_ramped_speed(t, rotation_time, ROTATION_SPEED)
                vz = maintain_altitude()
                self.publish_velocity_command(angular_z=w, linear_z=vz)
                rclpy.spin_once(self, timeout_sec=0.1)
            self.publish_velocity_command()
            time.sleep(1.0)

    def arm(self):
        req = ArmMotors.Request()
        req.arm = True
        future = self._client_arm.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def arm_with_timeout(self, timeout: rclpy.duration.Duration):
        armed = False
        start = self.get_clock().now()
        while not armed and self.get_clock().now() - start < timeout:
            armed = self.arm().result
            time.sleep(1)
        return armed

    def switch_mode(self, mode):
        req = ModeSwitch.Request()
        req.mode = mode
        future = self._client_mode_switch.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def switch_mode_with_timeout(self, desired_mode: int, timeout: rclpy.duration.Duration):
        ok = False
        start = self.get_clock().now()
        while not ok and self.get_clock().now() - start < timeout:
            result = self.switch_mode(desired_mode)
            ok = result.status or result.curr_mode == desired_mode
            time.sleep(1)
        return ok

def main(args=None):
    rclpy.init(args=args)
    node = CopterTakeoff()

    try:
        while node.curr_altitude == -1:
            rclpy.spin_once(node)

        if abs(node.curr_altitude - TAKEOFF_ALT) > 5:
            raise RuntimeError("Failed to reach takeoff altitude")

        if not node.switch_mode_with_timeout(COPTER_MODE_GUIDED, rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("Unable to switch to guided mode")

        while True:
            node.execute_square_movement()

        if not node.switch_mode_with_timeout(COPTER_MODE_RTL, rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("Unable to switch to RTL mode")

    except KeyboardInterrupt:
        node.publish_velocity_command()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
