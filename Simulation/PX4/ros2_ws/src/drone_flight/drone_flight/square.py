#!/usr/bin/env python3

import math
import rclpy
import numpy as np
from enum import Enum
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitude


class Phases(Enum):
    STARTING = 1
    LINEAR_MOVEMENT = 2
    LINEAR_MOVEMENT_END = 3
    ROTATING = 4
    ROTATION_END = 5


class OffboardControl(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.node_id = 0
        self.target_system = int(self.node_id) + 1
        node_name = "" if self.node_id == 0 else f"/px4_{self.node_id}"

        self.get_logger().info(f"Node name is {node_name} - sample topic : {node_name}/fmu/in/offboard_control_mode")

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f'{node_name}/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f'{node_name}/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f'{node_name}/fmu/in/vehicle_command', qos_profile)

        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, f'{node_name}/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, f'{node_name}/fmu/out/vehicle_attitude',
            self.attitude_callback, qos_profile)

        self.offboard_setpoint_counter = 0
        self.height_reached_post_time = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        self.loiter_height = -5.0
        self.initialized = False
        self.initial_position = (0, 0, 0)
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback_twist2)

        self.take_off_state = False
        self.obstacle_identified = False
        self.landing_initiated = False
        self.landing_timer = 0

        self.current_leg = None
        self.leg_start_time = None

        self.initial_heading = None
        self.vertical_velocity = -5.0
        self.linear_velocity = 0.0
        self.yaw = 0.0
        self.counter = 0
        self.state = Phases.STARTING
        self.state_start = 0
        self.trueYaw = 0.0

    def attitude_callback(self, msg):
        q = msg.q
        self.trueYaw = -np.arctan2(2.0 * (q[3] * q[0] + q[1] * q[2]),
                                   1.0 - 2.0 * (q[0] ** 2 + q[1] ** 2))

    def print_lidar_sensor_val(self, scan):
        validate_sample = lambda x: scan.range_min <= x <= scan.range_max
        true_hits = set(map(validate_sample, scan.ranges))
        if True in true_hits:
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
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_velocity_setpoint(self, vy=0.0):
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

    def takeoff(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=-5.0)
        self.get_logger().info("Takeoff command send")

    def publish_vehicle_command(self, command, **params) -> None:
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

    @staticmethod
    def tanh_scaling(x, a=2):
        return 5 * np.tanh(x / a)

    def set_vertical_velocity(self):
        self.vertical_velocity = self.tanh_scaling(-(self.vehicle_local_position.z - self.loiter_height))

    def timer_callback_twist2(self) -> None:
        self.publish_offboard_control_heartbeat_signal()
        self.set_vertical_velocity()
        self.offboard_setpoint_counter += 1

        if self.offboard_setpoint_counter < 10:
            return
        elif self.offboard_setpoint_counter == 10:
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

        self.initialized = True
        self.get_logger().info("\n\n final - Initialized \n\n")

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
