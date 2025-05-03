#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class OffboardControl(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.target_system = 1
        self.node_id = 0
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

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        self.initial_position = (0, 0, 0)
        self.initial_heading = None
        self.offboard_setpoint_counter = 0
        self.initialized = False
        self.takeoff_height = -5.0

        self.current_leg = None
        self.leg_start_time = None
        self.landing_timer = 0

        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback_twist2)

    def vehicle_local_position_callback(self, pos):
        self.vehicle_local_position = pos

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        msg.attitude = True
        msg.body_rate = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # 90 degrees in radians
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_velocity_setpoint(self, values):
        vx, vy, vz, yawspeed = values
        msg = TrajectorySetpoint()
        msg.velocity = [vx, vy, vz]
        msg.yawspeed = yawspeed
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Velocity setpoint: vx={vx}, vy={vy}, vz={vz}, yawspeed={yawspeed}")

    def publish_velocity_setpoint_twist(self, vx: float, vy: float, vz: float, yaw: float):
        msg = TrajectorySetpoint()
        msg.velocity = [vx, vy, vz]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Twist velocity setpoint: vx={vx}, vy={vy}, vz={vz}, yaw={yaw}")

    def publish_vehicle_command(self, command, **params) -> None:
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

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switched to offboard mode")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Landing initiated")

    def timer_callback_twist2(self):
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
            self.initial_heading = self.vehicle_local_position.heading

        if not (self.takeoff_height - 1 < self.vehicle_local_position.z < self.takeoff_height + 1) and not self.initialized:
            self.publish_velocity_setpoint([0.0, 0.0, -5.0, 0.0])
            return

        self.initialized = True
        self.get_logger().info("Drone initialized at target height.")

        if self.offboard_setpoint_counter > 500:
            self.publish_position_setpoint(*self.initial_position, self.takeoff_height)
            if abs(self.vehicle_local_position.x - self.initial_position[0]) < 0.2 and \
               abs(self.vehicle_local_position.y - self.initial_position[1]) < 0.2:
                self.land()
                exit(0)
            return

        rotation_angle = math.pi / 2
        current_heading = self.vehicle_local_position.heading
        remaining_angle = rotation_angle - (self.initial_heading - current_heading)
        self.publish_velocity_setpoint([0.0, 0.0, 0.0, 4.0])


def main(args=None) -> None:
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
        print(e)
