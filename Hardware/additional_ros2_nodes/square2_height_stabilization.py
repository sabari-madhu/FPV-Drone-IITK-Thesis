#!/usr/bin/env python3
"""
Square PID Node:
Continuously runs a square pattern with PID-controlled altitude.
This node has been adjusted so that its QoS settings match those used in
the provided sample nodes.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from ardupilot_msgs.srv import ModeSwitch
import math
import time

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Define a common QoS profile as used in the sample nodes.
default_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

# Copter Modes
COPTER_MODE_GUIDED = 4
COPTER_MODE_LOITER = 5
COPTER_MODE_RTL = 6

# Motion parameters
FORWARD_SPEED = 0.3  # m/s
ROTATION_SPEED = 0.5  # rad/s
SIDE_LENGTH = 5.0  # meters

class PIDController:
    def __init__(self, Kp, Ki, Kd, max_output):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_output = max_output

        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = time.time()

    def compute(self, target, current):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0.0:
            dt = 1e-6

        error = target - current
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output = max(min(output, self.max_output), -self.max_output)

        # Log PID computation details (if needed, consider using debug level in production)
        # Since PIDController is not a Node, the log is printed to stdout.
        print(f"[PID] Target: {target:.2f}, Current: {current:.2f}, Error: {error:.2f}, dt: {dt:.4f}, Output: {output:.2f}")

        self.prev_error = error
        self.last_time = now

        return output

class SquarePIDNode(Node):
    def __init__(self):
        super().__init__('square_pid_node')
        self.get_logger().info("Initializing SquarePIDNode...")

        self.curr_mode = None
        self.current_position = None
        self.current_yaw = None
        self.current_altitude = None
        self.stable_altitude = None

        self.state = 'FORWARD'
        self.start_position = None
        self.start_yaw = None
        self.side_completed = 0

        self.alt_pid = PIDController(Kp=0.9, Ki=0.0, Kd=0.3, max_output=2.0)

        # Use the same QoS settings for both publisher and subscription.
        self.create_subscription(PoseStamped, '/ap/pose/filtered', self.pose_callback, default_qos)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/ap/cmd_vel', default_qos)

        self.mode_client = self.create_client(ModeSwitch, '/ap/mode_switch')
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /ap/mode_switch...')

        self.create_timer(0.1, self.control_loop)
        self.create_timer(1.0, self.query_mode)
        self.get_logger().info("SquarePIDNode initialized and timers started.")

    def pose_callback(self, msg):
        self.current_position = msg.pose.position
        self.current_altitude = msg.pose.position.z
        self.current_yaw = self.quaternion_to_yaw(msg.pose.orientation)

        # Log the incoming pose information.
        self.get_logger().info(f"Received pose: Position (x: {self.current_position.x:.2f}, y: {self.current_position.y:.2f}, z: {self.current_altitude:.2f}), Yaw: {self.current_yaw:.2f} rad")

        if self.curr_mode in [COPTER_MODE_LOITER, COPTER_MODE_RTL]:
            self.stable_altitude = self.current_altitude
            self.get_logger().info(f"[Mode {self.curr_mode}] Stored stable altitude: {self.stable_altitude:.2f} m")

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # Log computed yaw from the quaternion.
        self.get_logger().debug(f"Computed yaw: {yaw:.2f} rad from quaternion")
        return yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop(self):
        if not all([self.current_position, self.current_yaw, self.current_altitude, self.stable_altitude]):
            self.get_logger().debug("Waiting for all necessary pose and altitude information before executing control loop.")
            return

        # Compute the altitude velocity correction using the PID controller.
        z_vel = self.alt_pid.compute(self.stable_altitude, self.current_altitude)
        self.get_logger().info(f"PID computed vertical velocity command: {z_vel:.2f} m/s")

        if self.state == 'FORWARD':
            if self.start_position is None:
                self.start_position = self.current_position
                self.get_logger().info("Start position set for FORWARD state.")
                return

            # Compute distance traveled from the start position.
            dx = self.current_position.x - self.start_position.x
            dy = self.current_position.y - self.start_position.y
            dist = math.sqrt(dx**2 + dy**2)
            self.get_logger().info(f"FORWARD state: Traveled distance = {dist:.2f} m")

            if dist >= SIDE_LENGTH:
                self.get_logger().info("Side length reached. Transitioning to ROTATE state.")
                self.state = 'ROTATE'
                self.start_yaw = self.current_yaw
                self.send_velocity_command(0.0, 0.0)
                return

            self.send_velocity_command(FORWARD_SPEED, z_vel)

        elif self.state == 'ROTATE':
            if self.start_yaw is None:
                self.start_yaw = self.current_yaw
                self.get_logger().info("Start yaw set for ROTATE state.")
                return

            # Calculate yaw difference and normalize it.
            delta_yaw = self.normalize_angle(self.current_yaw - self.start_yaw)
            self.get_logger().info(f"ROTATE state: Delta yaw = {delta_yaw:.2f} rad")

            if abs(delta_yaw) >= math.pi / 2:
                self.side_completed += 1
                self.get_logger().info(f"Completed rotation for side. Side count: {self.side_completed}")

                if self.side_completed >= 4:
                    self.get_logger().info("Completed one full square. Starting new square loop.")
                    self.side_completed = 0

                self.state = 'FORWARD'
                self.start_position = self.current_position
                self.start_yaw = None
                self.send_velocity_command(0.0, 0.0)
                return

            self.send_velocity_command(0.0, z_vel, angular_z=ROTATION_SPEED)

    def send_velocity_command(self, x=0.0, z=0.0, angular_z=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = x
        msg.twist.linear.z = z
        msg.twist.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f"Published velocity command: linear.x = {x:.2f}, linear.z = {z:.2f}, angular.z = {angular_z:.2f}")

    def _query_mode(self):
        self.get_logger().info("Querying current mode from /ap/mode_switch service...")
        req = ModeSwitch.Request()
        req.mode = 255  # dummy mode to just get current mode
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result:
            self.curr_mode = result.curr_mode
            self.get_logger().info(f"Current mode updated: {self.curr_mode}")
        else:
            self.get_logger().warning("Failed to get current mode from the service.")
            
    def query_mode(self):
        self.get_logger().info("Querying current mode from /ap/mode_switch service...")
        req = ModeSwitch.Request()
        req.mode = 255  # dummy mode to just get current mode
        future = self.mode_client.call_async(req)

        # Instead of spin_until_future_complete, add a done callback
        future.add_done_callback(self.handle_mode_response)

    def handle_mode_response(self, future):
        result = future.result()
        if result:
            self.curr_mode = result.curr_mode
            self.get_logger().info(f"Current mode updated: {self.curr_mode}")
        else:
            self.get_logger().warning("Failed to get current mode from the service.")

def main(args=None):
    rclpy.init(args=args)
    node = SquarePIDNode()
    try:
        node.get_logger().info("SquarePIDNode is spinning...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down node...")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
