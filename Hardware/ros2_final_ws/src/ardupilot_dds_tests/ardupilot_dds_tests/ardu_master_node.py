#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from ardupilot_msgs.srv import ModeSwitch
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import math
import time
import re

COPTER_MODE_GUIDED = 4
COPTER_MODE_LOITER = 5
COPTER_MODE_RTL = 6

default_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

class PIDController:
    """
    P - Proportional. Proportional to current error. The issue is when error is too small thrust becomes too less to make any meaningfull change and hence never reaches the target
    I - Integral. The accumulated error over time. Hence gives some thrust even if current error is less. But too much accumulation can lead to overshoot
    D - Derivative. Smoothens the motion. Reaching the target at pace -> change in error is negative and high -> larger damping effect. Falling from target fast -> positive slope and large -> boosting effect
    """
    def __init__(self, Kp, Ki, Kd, max_output):
        self.Kp = Kp # Proportional Gain - strenth of reaction to current error
        self.Ki = Ki # Integral Gain - The strength of the added up error over time
        self.Kd = Kd # Derivative Gain - responds to rate of change of error
        self.max_output = max_output # Keeps within this limit
        self.integral = 0.0 # Accumulates the total error over time
        self.prev_error = 0.0 # Just the previous error term for derivative computation
        self.last_time = time.time()

    def compute(self, target, current):

        # Measures the time between previous measurement and current
        now = time.time()
        dt = now - self.last_time
        if dt <= 0.0:
            dt = 1e-6

        # Computes the error between current height and required height
        error = target - current

        # Error over time. weighted by the time interval in which this error came in
        self.integral += error * dt

        # computes the rate of change of error - the slope
        derivative = (error - self.prev_error) / dt

        # Current error weighted by Proportional gain
        # Integral or error over time weighted by Integral gain
        # Rate of change of error weighted by Derivative gain
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Makes sure the output is between the +- max limits
        output = max(min(output, self.max_output), -self.max_output)

        self.prev_error = error
        self.last_time = now
        return output

class ArduMasterNode(Node):
    def __init__(self):
        super().__init__('ardu_master_node')
        self.get_logger().info("Initializing ArduMasterNode...")

        # Parameters
        self.declare_parameter("alt_pid_kp", 2.5)
        self.declare_parameter("alt_pid_ki", 0.0)
        self.declare_parameter("alt_pid_kd", 0.75)
        self.declare_parameter("alt_pid_max_output", 4.0)
        self.declare_parameter("linear_velocity", 1.0)
        self.declare_parameter("yaw_velocity", 1.0)

        # Altitude PID controller
        kp = self.get_parameter("alt_pid_kp").value
        ki = self.get_parameter("alt_pid_ki").value
        kd = self.get_parameter("alt_pid_kd").value
        max_out = self.get_parameter("alt_pid_max_output").value
        self.alt_pid = PIDController(kp, ki, kd, max_out)

        self.linear_velocity = 1.0 # self.get_parameter("linear_velocity").value
        self.yaw_velocity = self.get_parameter("yaw_velocity").value
        self.get_logger().info(f"Linear vel : {self.linear_velocity} and rot : {self.yaw_velocity}")

        self.curr_mode = COPTER_MODE_LOITER

        # Velocity command we’ll send to the flight controller
        self.current_velocity_cmd = TwistStamped()
        self.current_velocity_cmd.header.frame_id = 'base_link'

        # Subscribers
        self.pose_sub = self.create_subscription(PoseStamped, '/ap/pose/filtered', self.pose_callback, default_qos)
        self.movement = self.create_subscription(String, '/movement', self.movement_callback, default_qos)

        # Service client - to check which is the current mode
        self.mode_client = self.create_client(ModeSwitch, '/ap/mode_switch')
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /ap/mode_switch service...')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/ap/cmd_vel', default_qos)
        self.mode_pub = self.create_publisher(String, '/drone_mode', 10)
        self.guided_mode_activate = self.create_publisher(Bool, '/guided_mode_activation', 10)
        self.movement_completion = self.create_publisher(Bool, '/movement_completion', 10)


        # Timer to publish velocity to FC (e.g., 10 Hz)
        self.create_timer(0.1, self.publish_velocity)
        self.create_timer(1.0, self.query_mode)

        self.target_distance = 0
        self.target_yaw = 0
        self.start_position = 0
        self.start_yaw = 0
        self.movement_active = False
        self.yaw_complete = False

        self.get_logger().info("ArduMasterNode ready.")

    def movement_callback(self, msg: String):
        text = msg.data
        self.target_distance, self.target_yaw = map(float, re.findall(r'[-+]?\d*\.\d+|[-+]?\d+', text))
        self.target_yaw = math.radians(self.target_yaw)
        self.movement_active = True
        self.start_position = self.current_position
        self.start_yaw = self.current_yaw
        self.distance_to_travel = abs(self.target_distance)

        # Compute the final yaw target (assumed to be a relative yaw command)
        self.final_yaw = self.normalize_angle(self.start_yaw + self.target_yaw)
        self.get_logger().info(f"movement : target : {self.target_distance}, {self.target_yaw}. current : {self.start_position}, {self.current_yaw}. final yaw : {self.final_yaw}")

    def query_mode(self):
        """
        Call the Mode Service call with an invalid mode to get back the current mode
        """
        req = ModeSwitch.Request()
        req.mode = 255
        future = self.mode_client.call_async(req)
        future.add_done_callback(self.handle_mode_response)

    def handle_mode_response(self, future):
        """
        Callback to handle the mode change response
        """
        result = future.result()
        if result and self.curr_mode != result.curr_mode:
            self.movement_active = False
            self.movement_completion.publish(Bool(data=True))

            self.get_logger().info(f"Mode being switched to {result.curr_mode}")
            self.curr_mode = result.curr_mode

            # If the current mode is guided inform control node
            to_guided = False
            if self.curr_mode == COPTER_MODE_GUIDED: to_guided = True
            self.guided_mode_activate.publish(Bool(data=to_guided))

    def pose_callback(self, msg):
        self.current_position = msg.pose.position
        self.current_altitude = msg.pose.position.z
        self.current_yaw = self.quaternion_to_yaw(msg.pose.orientation)

        if self.curr_mode in [COPTER_MODE_LOITER, COPTER_MODE_RTL]:
            self.stable_altitude = self.current_altitude

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.get_logger().debug(f"Computed yaw: {yaw:.2f} rad from quaternion")
        return yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def publish_velocity(self):
        """Send the current velocity command to the flight controller."""
        # If we’re not in GUIDED, we can ignore
        if self.curr_mode != COPTER_MODE_GUIDED:
            return

        if self.movement_active:
            self.process_movement()

        z_vel = self.alt_pid.compute(self.stable_altitude, self.current_altitude)
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist.linear.z = z_vel
        twist.twist.linear.x = self.current_velocity_cmd.twist.linear.x
        twist.twist.angular.z = self.current_velocity_cmd.twist.angular.z
        self.cmd_vel_pub.publish(twist)

    def process_movement(self):

        # Compute the distance traveled and remaining
        dx = self.current_position.x - self.start_position.x

        # we can ignore the movement along y axis for now
        # dy = self.current_position.y - start_position.y
        dy = 0
        current_distance = math.sqrt(dx * dx + dy * dy)
        dist_remaining = self.distance_to_travel - current_distance

        # Compute yaw remaining dynamically:
        # (final_yaw - current_yaw) normalized to [-pi, pi].
        yaw_remaining = self.normalize_angle(self.final_yaw - self.current_yaw)

        if self.yaw_complete:
            yaw_remaining = 0

        self.get_logger().info(f"Executing remaining move: distance={dist_remaining}, yaw={yaw_remaining}")

        # Check if done
        if dist_remaining <= 0.5 and abs(yaw_remaining) <= 0.1:  # some threshold
            self.get_logger().info("Movement goal completed.")
            self.movement_active = False
            # Stop motion
            self.current_velocity_cmd.twist.linear.x = 0.0
            self.current_velocity_cmd.twist.angular.z = 0.0
            self.movement_completion.publish(Bool(data=True))
            self.yaw_complete = False
            return

        # Otherwise, set velocity
        # For simplicity, we either do distance OR yaw, not both simultaneously:

        # Set angular velocity to rotate towards the final yaw target.
        # Here we compute the sign of yaw_remaining to decide the direction.
        angular_speed = self.yaw_velocity

        if abs(yaw_remaining) > 0.1:
            yaw_sign = math.copysign(1, yaw_remaining)
            self.current_velocity_cmd.twist.angular.z = yaw_sign * angular_speed
            # We set just the yaw angle till yaw gets handled and only then handle the linear motion
            return
        else:
            self.current_velocity_cmd.twist.angular.z = 0.0
            self.yaw_complete = True

        if self.distance_to_travel > 0.0 and dist_remaining > 0.5:
            self.current_velocity_cmd.twist.linear.x = self.linear_velocity
        else:
            self.current_velocity_cmd.twist.linear.x = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = ArduMasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
