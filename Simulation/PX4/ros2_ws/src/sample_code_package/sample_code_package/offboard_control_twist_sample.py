#!/usr/bin/env python3

import math
import rclpy
import numpy as np
from enum import Enum
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, QoSReliabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleLocalPosition, VehicleStatus, VehicleAttitude
)

class Phases(Enum):
    STARTING = 1
    LINEAR_MOVEMENT = 2
    LINEAR_MOVEMENT_END = 3
    ROTATING = 4
    ROTATION_END = 5

class OffboardControl(Node):
    """Node for offboard drone control with linear movement and yaw rotation."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Parameters
        self.declare_parameter('node_id', 1)
        self.declare_parameter('num_nodes', 5)
        self.declare_parameter('model_name', 'iris')

        self.node_id = self.get_parameter('node_id').get_parameter_value().integer_value
        self.n_drones = self.get_parameter('num_nodes').get_parameter_value().integer_value
        model_name = self.get_parameter('model_name').get_parameter_value().string_value

        namespace = f"{model_name}{self.node_id}"
        lidar_topic = f"/{namespace}/scan"
        node_name = "" if self.node_id == 0 else f"/px4_{self.node_id}"
        self.target_system = self.node_id + 1

        self.get_logger().info(f"Lidar topic: {lidar_topic}")
        self.get_logger().info(f"Node name: {node_name}")

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f'{node_name}/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f'{node_name}/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f'{node_name}/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.create_subscription(LaserScan, lidar_topic, self.print_lidar_sensor_val,
                                 QoSProfile(depth=10,
                                            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT))
        self.create_subscription(VehicleLocalPosition, f'{node_name}/fmu/out/vehicle_local_position',
                                 self.vehicle_local_position_callback, qos_profile)
        self.create_subscription(VehicleAttitude, f'{node_name}/fmu/out/vehicle_attitude',
                                 self.attitude_callback, qos_profile)

        # Internal State
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.trueYaw = 0.0
        self.initial_heading = None
        self.initial_position = (0, 0, 0)
        self.loiter_height = -5.0
        self.vertical_velocity = -5.0
        self.linear_velocity = 0.0
        self.yaw = 0.0
        self.obstacle_identified = False

        # State Control
        self.offboard_setpoint_counter = 0
        self.initialized = False
        self.state = Phases.STARTING
        self.counter = 0

        # Timer
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback_twist2)

    # ---------------------- Callbacks ----------------------

    def attitude_callback(self, msg):
        q = msg.q
        self.trueYaw = -np.arctan2(2.0 * (q[3]*q[0] + q[1]*q[2]),
                                   1.0 - 2.0 * (q[0]**2 + q[1]**2))

    def vehicle_local_position_callback(self, pos):
        self.vehicle_local_position = pos

    def print_lidar_sensor_val(self, scan):
        valid = lambda x: scan.range_min <= x <= scan.range_max
        if True in map(valid, scan.ranges):
            self.obstacle_identified = True

    # ---------------------- Command Helpers ----------------------

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("Disarm command sent")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to Offboard mode")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Landing...")

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    # ---------------------- Motion ----------------------

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.velocity = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def tanh_scaling(self, x, a=2):
        return 5 * np.tanh(x / a)

    def set_vertical_velocity(self):
        self.vertical_velocity = self.tanh_scaling(-(self.vehicle_local_position.z - self.loiter_height))
        self.get_logger().info(f"Altitude: {self.vehicle_local_position.z:.2f} -> Vz: {self.vertical_velocity:.2f}")

    def publish_velocity_setpoint(self):
        cos_yaw = np.cos(self.trueYaw)
        sin_yaw = np.sin(self.trueYaw)
        vx = -(self.linear_velocity * cos_yaw)
        vy = -(self.linear_velocity * sin_yaw)

        msg = TrajectorySetpoint()
        msg.velocity = [vx, vy, self.vertical_velocity]
        msg.yawspeed = self.yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.yaw = float('nan')
        self.trajectory_setpoint_publisher.publish(msg)

    # ---------------------- Timer Callback ----------------------

    def timer_callback_twist2(self):
        self.publish_offboard_control_heartbeat_signal()
        self.set_vertical_velocity()
        self.offboard_setpoint_counter += 1

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.initial_position = (
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z
            )
            self.initial_heading = self.vehicle_local_position.heading

        if not (self.loiter_height - 1 < self.vehicle_local_position.z < self.loiter_height + 1) and not self.initialized:
            self.publish_velocity_setpoint()
            return

        if not self.initialized:
            self.initialized = True
            self.get_logger().info("\n\n Final - Initialized \n\n")

        self.counter += 1

        if self.state == Phases.STARTING:
            self.state = Phases.LINEAR_MOVEMENT
            self.counter = 0

        elif self.state == Phases.LINEAR_MOVEMENT:
            self.linear_velocity = 1.0
            if self.counter <= (5 / self.timer_period):
                self.publish_velocity_setpoint()
                return
            self.state = Phases.LINEAR_MOVEMENT_END
            self.counter = 0

        elif self.state == Phases.LINEAR_MOVEMENT_END:
            self.linear_velocity = 0.0
            self.state = Phases.ROTATING
            self.counter = 0

        elif self.state == Phases.ROTATING:
            self.yaw = 1.0
            if self.counter <= ((np.pi / 2) / self.timer_period):
                self.publish_velocity_setpoint()
                return
            self.state = Phases.ROTATION_END
            self.counter = 0
            self.yaw = 0.0

        elif self.state == Phases.ROTATION_END:
            self.state = Phases.STARTING
            self.counter = 0

        self.publish_velocity_setpoint()


def main(args=None):
    print("Starting offboard control node...")
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
