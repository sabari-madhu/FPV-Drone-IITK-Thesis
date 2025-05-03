#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, QoSReliabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus
)


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # QoS profile setup
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Declare parameters
        self.declare_parameter('node_id', 1)
        self.declare_parameter('num_nodes', 5)
        self.declare_parameter('model_name', 'iris')

        # Read parameters
        self.node_id = self.get_parameter('node_id').get_parameter_value().integer_value
        self.n_drones = self.get_parameter('num_nodes').get_parameter_value().integer_value
        model_name = self.get_parameter('model_name').get_parameter_value().string_value

        # Lidar topic subscription
        namespace = f"{model_name}{self.node_id}"
        lidar_topic = f"/{namespace}/scan"
        self.get_logger().info(f"Lidar topic : {lidar_topic}")
        self.getTargetScan_ = self.create_subscription(
            LaserScan,
            lidar_topic,
            self.print_lidar_sensor_val,
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        )

        # PX4 topic setup
        self.target_system = self.node_id + 1
        node_name = "" if self.node_id == 0 else f"/px4_{self.node_id}"
        self.get_logger().info(f"Node name is {node_name}")

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f'{node_name}/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f'{node_name}/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f'{node_name}/fmu/in/vehicle_command', qos_profile)

        # Internal state
        self.offboard_setpoint_counter = 0
        self.height_reached_post_time = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        self.initialized = False
        self.initial_position = (0, 0, 0)
        self.timer_period = 0.5
        self.take_off_state = False
        self.obstacle_identified = False
        self.landing_initiated = False
        self.landing_timer = 0

        # Timer
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def print_lidar_sensor_val(self, scan):
        """Handle incoming lidar data and check for obstacle presence."""
        is_valid = lambda x: scan.range_min <= x <= scan.range_max
        hits = set(map(is_valid, scan.ranges))
        if True in hits:
            self.obstacle_identified = True

    def vehicle_local_position_callback(self, pos):
        self.vehicle_local_position = pos

    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

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
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

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

        if self.landing_initiated:
            self.landing_timer += 1
            if self.landing_timer > 40:
                self.landing_timer = 0
                self.landing_initiated = False
                self.get_logger().info("Ready to fly again!")
            return

        if not self.obstacle_identified:
            return

        if not self.take_off_state:
            self.engage_offboard_mode()
            self.arm()
            self.initial_position = (
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z
            )
            self.take_off_state = True
            return

        self.offboard_setpoint_counter += 1

        if (self.vehicle_local_position.z - 0.5 > self.takeoff_height and
                self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and not self.initialized):
            self.publish_position_setpoint(*self.initial_position[:2], self.takeoff_height)
            return

        self.initialized = True

        if self.offboard_setpoint_counter > 100:
            self.get_logger().info("Landing Initiated")
            self.publish_position_setpoint(*self.initial_position[:2], self.takeoff_height)

            if (abs(self.vehicle_local_position.x - self.initial_position[0]) < 0.2 and
                abs(self.vehicle_local_position.y - self.initial_position[1]) < 0.2):
                self.land()
                self.obstacle_identified = False
                self.take_off_state = False
                self.landing_initiated = True
                self.landing_timer += 1
                self.offboard_setpoint_counter = 0
            return

        # Circle motion
        radius = 10.0
        angular_velocity = 0.9
        current_time = self.offboard_setpoint_counter * self.timer_period
        phase_shift = (2 * math.pi / self.n_drones) * self.node_id

        x = self.initial_position[0] + radius * math.cos(angular_velocity * current_time + phase_shift)
        y = self.initial_position[1] + radius * math.sin(angular_velocity * current_time + phase_shift)
        z = self.takeoff_height

        self.publish_position_setpoint(x, y, z)


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
