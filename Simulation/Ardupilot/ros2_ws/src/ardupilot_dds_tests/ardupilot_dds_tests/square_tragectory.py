#!/usr/bin/env python3
# Copyright 2023 ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

"""
Run takeoff test on Copter with movement sequence and RTL.

Warning - This is NOT production code; it's a simple demo of capability.
"""

import rclpy
import time
import numpy as np

from rclpy.node import Node
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import TwistStamped, PoseStamped
from ardupilot_msgs.srv import ArmMotors
from ardupilot_msgs.srv import ModeSwitch
from ardupilot_msgs.srv import Takeoff


COPTER_MODE_GUIDED = 4
COPTER_MODE_RTL = 6
COPTER_MODE_TAKEOFF = 13  # Add this with your other mode constants

# This must be a float value
TAKEOFF_ALT = 4.0
MOVEMENT_TIME = 5  # seconds for each movement
HOVER_TIME = 5  # seconds for final hover

# Movement parameters
FORWARD_SPEED = 1.0  # m/s
ROTATION_SPEED = 0.5  # rad/s


class CopterTakeoff(Node):
    """Copter takeoff using guided control."""

    def __init__(self):
        """Initialise the node."""
        super().__init__("copter_takeoff")

        self.declare_parameter("arm_topic", "/ap/arm_motors")
        self._arm_topic = self.get_parameter("arm_topic").get_parameter_value().string_value
        self._client_arm = self.create_client(ArmMotors, self._arm_topic)
        while not self._client_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arm service not available, waiting again...')

        self.declare_parameter("mode_topic", "/ap/mode_switch")
        self._mode_topic = self.get_parameter("mode_topic").get_parameter_value().string_value
        self._client_mode_switch = self.create_client(ModeSwitch, self._mode_topic)
        while not self._client_mode_switch.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('mode switch service not available, waiting again...')

        # self.declare_parameter("takeoff_service", "/ap/experimental/takeoff")
        # self._takeoff_topic = self.get_parameter("takeoff_service").get_parameter_value().string_value
        # self._client_takeoff = self.create_client(Takeoff, self._takeoff_topic)
        # while not self._client_takeoff.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('takeoff service not available, waiting again...')

        self.declare_parameter("geopose_topic", "/ap/geopose/filtered")
        self._geopose_topic = self.get_parameter("geopose_topic").get_parameter_value().string_value
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, durability=rclpy.qos.DurabilityPolicy.VOLATILE, depth=1
        )

        self._subscription_geopose = self.create_subscription(GeoPoseStamped, self._geopose_topic, self.geopose_cb, qos)
        self._cur_geopose = GeoPoseStamped()

        # Initialize velocity command publisher
        self._cmd_vel_pub = self.create_publisher(TwistStamped, '/ap/cmd_vel', 10)

        self.declare_parameter("localpose_topic", "/ap/pose/filtered")
        self._localpose_topic = self.get_parameter("localpose_topic").get_parameter_value().string_value
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, durability=rclpy.qos.DurabilityPolicy.VOLATILE, depth=1
        )

        self._subscription_localpose = self.create_subscription(PoseStamped, self._localpose_topic, self.localpose_cb, qos)
        self._cur_localpose = PoseStamped()

        self.curr_altitude = -1

    def localpose_cb(self, msg: PoseStamped):
        """Process a GeoPose message."""
        stamp = msg.header.stamp
        self.get_logger().info(f"Height from pose : {self.curr_altitude}") 
        if stamp.sec:
            self.get_logger().info("From AP : Geopose [sec:{}, nsec: {}]".format(stamp.sec, stamp.nanosec))

            # Store current state
            self._cur_localpose = msg.pose.position

            self.curr_altitude = self._cur_localpose.z

            self.get_logger().info(f"Height from pose : {self.curr_altitude}")                

    def publish_velocity_command(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, 
                               angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """Publish velocity command."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Linear velocity
        msg.twist.linear.x = linear_x
        msg.twist.linear.y = linear_y
        msg.twist.linear.z = linear_z
        
        # Angular velocity
        msg.twist.angular.x = angular_x
        msg.twist.angular.y = angular_y
        msg.twist.angular.z = angular_z
        
        self._cmd_vel_pub.publish(msg)

    def execute_movement_sequence(self):
        """Execute the movement sequence."""
        # Move forward
        self.get_logger().info("Moving forward...")
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time) < rclpy.duration.Duration(seconds=MOVEMENT_TIME):
            self.publish_velocity_command(linear_x=FORWARD_SPEED)
            time.sleep(0.1)  # Publish at 10Hz
        
        # Stop forward movement
        self.publish_velocity_command()
        
        # Rotate
        self.get_logger().info("Rotating...")
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time) < rclpy.duration.Duration(seconds=MOVEMENT_TIME):
            self.publish_velocity_command(angular_z=ROTATION_SPEED)
            time.sleep(0.1)  # Publish at 10Hz
        
        # Stop rotation
        self.publish_velocity_command()
        
        # Hover
        self.get_logger().info(f"Hovering for {HOVER_TIME} seconds...")
        time.sleep(HOVER_TIME)

    def execute_square_movement(self):
        """Execute a square movement pattern."""
        # Side length and total sides
        SIDE_LENGTH = 5  # meters
        NUM_SIDES = 4
        
        for _ in range(NUM_SIDES):
            # Move forward for one side
            self.get_logger().info(f"Moving forward {SIDE_LENGTH} meters...")
            start_time = self.get_clock().now()
            while (self.get_clock().now() - start_time) < rclpy.duration.Duration(seconds=SIDE_LENGTH/FORWARD_SPEED):
                self.publish_velocity_command(linear_x=FORWARD_SPEED)
                rclpy.spin_once(self, timeout_sec=0.1)
            
            # Stop forward movement
            self.publish_velocity_command()
            
            # Rotate 90 degrees
            self.get_logger().info("Rotating 90 degrees...")
            start_time = self.get_clock().now()
            while (self.get_clock().now() - start_time) < rclpy.duration.Duration(seconds=np.pi/(2*ROTATION_SPEED)):
                self.publish_velocity_command(angular_z=ROTATION_SPEED)
                rclpy.spin_once(self, timeout_sec=0.1)
            
            # Stop rotation
            self.publish_velocity_command()        

    def geopose_cb(self, msg: GeoPoseStamped):
        """Process a GeoPose message."""
        stamp = msg.header.stamp
        if stamp.sec:
            self.get_logger().info("From AP : Geopose [sec:{}, nsec: {}]".format(stamp.sec, stamp.nanosec))

            # Store current state
            self._cur_geopose = msg

    def wait_for_takeoff(self, target_altitude):
        """Wait for drone to reach target altitude while keeping ROS2 event loop active."""
        takeoff_complete = False
        
        def timer_callback():
            nonlocal takeoff_complete
            current_alt = self.get_curr_alt()
            self.get_logger().info(f"Current altitude: {current_alt}")
            
            if current_alt >= target_altitude:
                takeoff_complete = True
                self.get_logger().info("Reached takeoff altitude!")

        # Create a timer that checks altitude every second
        timer = self.create_timer(1.0, timer_callback)
        
        # Spin until takeoff is complete
        while rclpy.ok() and not takeoff_complete:
            rclpy.spin_once(self, timeout_sec=1.0)
        
        # Destroy the timer
        timer.destroy()            

    def arm(self):
        req = ArmMotors.Request()
        req.arm = True
        future = self._client_arm.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def arm_with_timeout(self, timeout: rclpy.duration.Duration):
        """Try to arm. Returns true on success, or false if arming fails or times out."""
        armed = False
        start = self.get_clock().now()
        while not armed and self.get_clock().now() - start < timeout:
            armed = self.arm().result
            time.sleep(1)
        return armed

    def switch_mode(self, mode):
        req = ModeSwitch.Request()
        assert mode in [COPTER_MODE_GUIDED, COPTER_MODE_RTL, COPTER_MODE_TAKEOFF]
        req.mode = mode
        future = self._client_mode_switch.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def switch_mode_with_timeout(self, desired_mode: int, timeout: rclpy.duration.Duration):
        """Try to switch mode. Returns true on success, or false if mode switch fails or times out."""
        is_in_desired_mode = False
        start = self.get_clock().now()
        while not is_in_desired_mode and self.get_clock().now() - start < timeout:
            result = self.switch_mode(desired_mode)
            # Handle successful switch or the case that the vehicle is already in expected mode
            is_in_desired_mode = result.status or result.curr_mode == desired_mode
            time.sleep(1)

        return is_in_desired_mode

    def takeoff(self, alt):
        req = Takeoff.Request()
        req.alt = alt
        future = self._client_takeoff.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def takeoff_with_timeout(self, alt: int, timeout: rclpy.duration.Duration):
        """Try to takeoff. Returns true on success, or false if takeoff fails or times out."""
        takeoff_success = False
        start = self.get_clock().now()
        while not takeoff_success and self.get_clock().now() - start < timeout:
            result = self.takeoff(alt)
            takeoff_success = result.status
            time.sleep(1)

        return takeoff_success

    def get_cur_geopose(self):
        """Return latest geopose."""
        return self._cur_geopose

    def get_curr_alt(self):
        return self.curr_altitude        


def main(args=None):
    """Node entry point."""
    rclpy.init(args=args)
    node = CopterTakeoff()

    try:
        
        # Block till armed, which will wait for EKF3 to initialize
        # if not node.arm_with_timeout(rclpy.duration.Duration(seconds=30)):
        #     raise RuntimeError("Unable to arm")

        # node.get_logger().info("Armed!")

        # if not node.switch_mode_with_timeout(COPTER_MODE_TAKEOFF, rclpy.duration.Duration(seconds=20)):
        #     raise RuntimeError("Unable to switch to takeoff mode")

        # node.get_logger().info("Takeoff mode initiated!")

        # is_ascending_to_takeoff_alt = True
        # while is_ascending_to_takeoff_alt:
        #     rclpy.spin_once(node)
        #     time.sleep(1.0)
        #     is_ascending_to_takeoff_alt = node.get_curr_alt() < TAKEOFF_ALT

        # if is_ascending_to_takeoff_alt:
        #     raise RuntimeError("Failed to reach takeoff altitude")        

        if not node.switch_mode_with_timeout(COPTER_MODE_GUIDED, rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("Unable to switch to guided mode")

        node.get_logger().info("Mode Switch to guided completed!")

        # Execute movement sequence
        # node.execute_movement_sequence()
        node.execute_square_movement()

        # Switch to RTL mode
        node.get_logger().info("Switching to RTL mode...")
        if not node.switch_mode_with_timeout(COPTER_MODE_RTL, rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("Unable to switch to RTL mode")
        
        node.get_logger().info("Successfully switched to RTL mode. Mission complete.")

    except KeyboardInterrupt:
        # Ensure we stop any ongoing movement
        node.publish_velocity_command()
        pass

    # Destroy the node explicitly.
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()