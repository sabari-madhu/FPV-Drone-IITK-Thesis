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
    """Node for controlling a PX4-based vehicle in offboard mode for takeoff, circular flight, and landing."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # QoS profile setup
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Parameters
        self.declare_parameter('node_id', 1)
        self.declare_parameter('num_nodes', 5)

        self.node_id = self.get_parameter('node_id').get_parameter_value().integer_value
        self.n_drones = self.get_parameter('num_nodes').get_parameter_value().integer_value
        self.target_system = self.node_id + 1

        node_name = "" if self.node_id == 0 else f"/px4_{self.node_id}"
        self.get_logger().info(f"Node namespace: {node_name}, Target System: {self.target_system}")

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f'{node_name}/fmu/in/offboard_control_mode', qos_profile
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f'{node_name}/fmu/in/trajectory_setpoint', qos_profile
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f'{node_name}/fmu/in/vehicle_command', qos_profile
        )

        # Subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, f'{node_name}/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile
        )
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, f'{node_name}/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile
        )

        # Internal state
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.initial_position = (0, 0, 0)
        self.takeoff_height = -5.0  # Negative Z = upward
        self.initialized = False
        self.timer_period = 0.5

        # Control loop timer
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    # Callbacks
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    # Vehicle commands
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switched to Offboard mode")

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
        self.get_logger().info(f"Position setpoint published: ({x:.2f}, {y:.2f}, {z:.2f})")

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
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self):
        self.publish_offboard_control_heartbeat_signal()
        self.offboard_setpoint_counter += 1

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

        if not self.initialized:
            self.get_logger().info("Drone has reached takeoff height. Starting circular flight.")
            self.initialized = True

        # Land after some time
        if self.offboard_setpoint_counter > 100:
            self.get_logger().info("Returning to initial position for landing...")
            self.publish_position_setpoint(*self.initial_position[:2], self.takeoff_height)
            if (abs(self.vehicle_local_position.x - self.initial_position[0]) < 0.2 and
                abs(self.vehicle_local_position.y - self.initial_position[1]) < 0.2):
                self.land()
                exit(0)
            return

        # Circle flight logic with phase shift per drone
        radius = 10.0
        angular_velocity = 0.9
        current_time = self.offboard_setpoint_counter * self.timer_period
        phase_shift = (2 * math.pi / self.n_drones) * self.node_id

        x = self.initial_position[0] + radius * math.cos(angular_velocity * current_time + phase_shift)
        y = self.initial_position[1] + radius * math.sin(angular_velocity * current_time + phase_shift)
        z = self.takeoff_height
        self.publish_position_setpoint(x, y, z)


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
