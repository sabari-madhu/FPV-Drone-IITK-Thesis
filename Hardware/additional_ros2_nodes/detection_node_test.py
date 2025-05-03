#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# same QoS as your ArduMasterNode
default_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

class BoolDetectionNode(Node):
    def __init__(self):
        super().__init__('bool_detection_node')
        self.get_logger().info('Initializing BoolDetectionNode...')

        # Internal state flags
        self.guided_mode_active = False
        self.movement_completed = True  # allow first command

        # Subscriber: guided‑mode activation
        self.create_subscription(Bool,
                                 '/guided_mode_activation',
                                 self.guided_mode_callback,
                                 default_qos)

        # Subscriber: movement completion flag
        self.create_subscription(Bool,
                                 '/movement_completion',
                                 self.movement_complete_callback,
                                 default_qos)

        # Subscriber: your boolean “detection” input
        self.create_subscription(Bool,
                                 '/detection_input',
                                 self.detection_callback,
                                 default_qos)

        # Publisher: send movement commands
        self.movement_pub = self.create_publisher(String,
                                                  '/movement',
                                                  default_qos)

        # max yaw deflection, same parameter
        self.declare_parameter("max_angle_rotation", 60.0)
        self.max_angle_rotation = self.get_parameter("max_angle_rotation").value

        self.get_logger().info('BoolDetectionNode ready.')

    def guided_mode_callback(self, msg: Bool):
        self.guided_mode_active = msg.data
        self.get_logger().info(f'Guided mode: {self.guided_mode_active}')

    def movement_complete_callback(self, msg: Bool):
        self.movement_completed = msg.data
        self.get_logger().info(f'Movement done: {self.movement_completed}')

    def detection_callback(self, msg: Bool):
        # only act if guided and previous movement done
        if not self.guided_mode_active or not self.movement_completed:
            return

        self.movement_completed = False
        response = String()

        # treat msg.data as your “detected” flag
        if msg.data:
            # detection → move forward with zero yaw
            response.data = f'linear_2.0_yaw_-15.0'
            self.get_logger().info('Detected: publishing forward command')
        else:
            # no detection → e.g. rotate in place
            yaw = -30
            response.data = f'linear_0.0_yaw_{yaw}'
            self.get_logger().info('Not detected: publishing rotate command')

        self.movement_pub.publish(response)

def main(args=None):
    rclpy.init(args=args)
    node = BoolDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
