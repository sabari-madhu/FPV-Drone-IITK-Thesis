#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ArdupilotCommunicator node
        Node(
            package='thesis_nodes',
            executable='ardupilot_communicator',
            name='ardupilot_communicator'
        ),
        # Scan node (publishing on /internal/cmd_vel_scan)
        Node(
            package='thesis_nodes',
            executable='scan',
            name='scan'
        ),
        # CameraModule node (with object detection)
        Node(
            package='thesis_nodes',
            executable='camera_module',
            name='camera_module'
        ),
        # Tracking node (publishing on /internal/cmd_vel_tracking)
        Node(
            package='thesis_nodes',
            executable='tracking',
            name='tracking'
        ),
        # MasterDrone node
        Node(
            package='thesis_nodes',
            executable='master_drone',
            name='master_drone'
        )
    ])
