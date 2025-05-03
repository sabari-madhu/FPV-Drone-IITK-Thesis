#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from geographic_msgs.msg import GeoPoseStamped
from ardupilot_msgs.srv import ModeSwitch

class ArdupilotCommunicator(Node):
    def __init__(self):
        super().__init__('ardupilot_communicator')
        # Forward velocity commands: internal -> Ardupilot
        self._ap_cmd_vel_pub = self.create_publisher(TwistStamped, '/ap/cmd_vel', 10)
        self.create_subscription(TwistStamped, '/internal/cmd_vel', self.internal_cmd_vel_callback, 10)
        
        # Mode switch: client to Ardupilot and an internal service for mode switching
        self._mode_switch_client = self.create_client(ModeSwitch, '/ap/mode_switch')
        while not self._mode_switch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /ap/mode_switch service...")
        self.create_service(ModeSwitch, '/internal/mode_switch', self.internal_mode_switch_callback)
        
        # Geopose republishing: subscribe to Ardupilot and republish internally
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, durability=rclpy.qos.DurabilityPolicy.VOLATILE, depth=1
        )
        self.create_subscription(GeoPoseStamped, '/ap/geopose/filtered', self.geopose_callback, qos)
        self._internal_geopose_pub = self.create_publisher(GeoPoseStamped, '/internal/geopose', 10)
        
        self.get_logger().info("ArdupilotCommunicator node started.")

    def internal_cmd_vel_callback(self, msg: TwistStamped):
        self.get_logger().debug(f"Forwarding velocity command to /ap/cmd_vel. :             {msg}")
        self._ap_cmd_vel_pub.publish(msg)

    def internal_mode_switch_callback(self, request, response):
        self.get_logger().debug(f"Internal mode switch request received: mode={request.mode}")
        future = self._mode_switch_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is not None:
            response.status = result.status
            response.curr_mode = result.curr_mode
            self.get_logger().debug(f"Mode switched: status={result.status}, curr_mode={result.curr_mode}")
        else:
            self.get_logger().error("Failed to call Ardupilot mode switch service.")
            response.status = False
            response.curr_mode = -1
        return response

    def geopose_callback(self, msg: GeoPoseStamped):
        self.get_logger().debug("Republishing geopose data to /internal/geopose.")
        self._internal_geopose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArdupilotCommunicator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ArdupilotCommunicator node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
