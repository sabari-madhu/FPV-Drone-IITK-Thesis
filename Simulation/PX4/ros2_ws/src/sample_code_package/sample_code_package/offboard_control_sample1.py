#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus
)


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode with takeoff, circular motion, and landing."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)

        # Internal state
        self.offboard_setpoint_counter = 0
        self.height_reached_post_time = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.initial_position = (0, 0, 0)
        self.takeoff_height = -5.0  # Negative Z = upward
        self.initialized = False

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    # Callbacks
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    # PX4 commands
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Landing initiated")

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = msg.acceleration = msg.attitude = msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # 90 degrees
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing setpoint: [{x:.2f}, {y:.2f}, {z:.2f}]")

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    # Main control loop
    def timer_callback(self):
        self.publish_offboard_control_heartbeat_signal()
        self.offboard_setpoint_counter += 1

        if self.offboard_setpoint_counter < 10:
            return

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.initial_position = (
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z
            )

        # Continue takeoff until target height is reached
        if (self.vehicle_local_position.z - 0.5 > self.takeoff_height and
            self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and
            not self.initialized):
            self.publish_position_setpoint(*self.initial_position[:2], self.takeoff_height)
            return

        self.initialized = True

        # Circle flight
        radius = 5.0
        angular_velocity = 0.1
        current_time = self.height_reached_post_time * 0.1
        x = self.initial_position[0] + radius * math.cos(angular_velocity * current_time)
        y = self.initial_position[1] + radius * math.sin(angular_velocity * current_time)
        z = self.takeoff_height

        self.height_reached_post_time += 1
        self.publish_position_setpoint(x, y, z)

        if self.offboard_setpoint_counter > 1000:
            self.land()
            exit(0)


def main(args=None):
    print('Starting offboard control node...')
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")
