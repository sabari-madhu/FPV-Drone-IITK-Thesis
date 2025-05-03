#!/usr/bin/env python3

import os
import subprocess
import rclpy
from rclpy.node import Node
from pathlib import Path
from jinja2 import Environment, FileSystemLoader
from gazebo_msgs.srv import SpawnEntity

SUPPORTED_MODELS = ["iris", "iris_custom", "plane", "standard_vtol", "rover", "r1_rover", "typhoon_h480"]


def generate_sdf(template_path, output_path, context):
    env = Environment(loader=FileSystemLoader(os.path.dirname(template_path)))
    template = env.get_template(os.path.basename(template_path))
    sdf_content = template.render(context)

    with open(output_path, 'w') as sdf_file:
        sdf_file.write(sdf_content)


def start_px4_instance(build_path, instance_num):
    working_dir = os.path.join(build_path, 'rootfs', str(instance_num))
    os.makedirs(working_dir, exist_ok=True)

    cmd = [os.path.join(build_path, 'bin', 'px4'), '-i', str(instance_num), '-d', os.path.join(build_path, 'etc')]
    with open(os.path.join(working_dir, 'out.log'), 'w') as out_log, open(os.path.join(working_dir, 'err.log'), 'w') as err_log:
        subprocess.Popen(cmd, stdout=out_log, stderr=err_log, cwd=working_dir)


def spawn_model(node, client, model_name, instance_num, sdf_file, x, y):
    request = SpawnEntity.Request()
    request.name = f"{model_name}_{instance_num}"
    request.robot_namespace = f"/{model_name}_{instance_num}"
    request.xml = open(sdf_file, 'r').read()
    request.initial_pose.position.x = x
    request.initial_pose.position.y = y
    request.initial_pose.position.z = 0.0

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info(f"Spawned {request.name} at ({x}, {y})")
    else:
        raise RuntimeError(f"Failed to spawn model: {future.exception()}")


def main():
    rclpy.init()
    node = rclpy.create_node('drone_spawner')

    node.declare_parameter('model', 'iris')
    node.declare_parameter('num_instances', 2)

    model = node.get_parameter('model').get_parameter_value().string_value
    num_instances = node.get_parameter('num_instances').get_parameter_value().integer_value

    if model not in SUPPORTED_MODELS:
        node.get_logger().error(f"Model '{model}' not supported. Supported: {SUPPORTED_MODELS}")
        rclpy.shutdown()
        return

    client = node.create_client(SpawnEntity, '/spawn_entity')
    node.get_logger().info("Waiting for spawn_entity service...")
    client.wait_for_service()
    node.get_logger().info("Service available. Starting...")

    build_path = '/home/sabari/Workspace/Thesis/PX4/PX4-Autopilot/build/px4_sitl_default'
    src_path = '/home/sabari/Workspace/Thesis/PX4/PX4-Autopilot/'

    for i in range(1, num_instances + 1):
        x = 0.0
        y = float(3 * i)

        sdf_template_path = os.path.join(src_path, 'Tools', 'simulation', 'gazebo-classic', 'sitl_gazebo-classic',
                                         'models', model, f'{model}.sdf.jinja')
        output_sdf_path = f'/tmp/{model}_{i}.sdf'

        context = {
            'mavlink_tcp_port': 4560 + i,
            'mavlink_udp_port': 14560 + i,
            'mavlink_id': 1 + i,
            'gst_udp_port': 5600 + i,
            'video_uri': 5600 + i,
            'mavlink_cam_udp_port': 14530 + i,
        }

        generate_sdf(sdf_template_path, output_sdf_path, context)
        node.get_logger().info(f"SDF generated for {model}_{i}")
        start_px4_instance(build_path, i)
        node.get_logger().info(f"PX4 instance {i} started")
        spawn_model(node, client, model, i, output_sdf_path, x, y)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
