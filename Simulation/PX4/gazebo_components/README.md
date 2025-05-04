# Gazebo Simulation Models for PX4 ROS2 Integration

This folder contains customized Gazebo Classic simulation models tailored for integration with the PX4 autopilot and ROS2-based drone control systems. These assets support experiments in autonomous navigation, multi-drone coordination, and sensor-based perception in a simulated environment.

## Folder Structure and Components

*   `iris_custom/`

    *   A modified version of the default PX4 Iris drone model.
    *   Includes a `.sdf.jinja` template to enable flexible spawning via ROS2-based scripts.
    *   Updated mesh files for use with simulation.

*   `iris_rplidar_custom/`

    *   Combines the Iris drone with a custom RPLIDAR sensor.
    *   Designed for tasks involving 3D perception, SLAM, or target tracking in GPS-denied scenarios.

*   `rplidar_custom/` and `lidar_updated/`

    *   Standalone LIDAR sensor models used for integration on aerial platforms.
    *   These models have been adapted from the PX4 Gazebo repo to suit specific sensor configurations.

*   `custom_box/`

    *   A static object model with a texture and material script.
    *   Useful for object detection tasks and environment enrichment during simulation.

*   `sitl_multiple_run_edited.sh`

    *   Modified SITL launch script that enables spawning and controlling drones from ROS2 nodes.
    *   Originally used for launching PX4 SITL instances directly from the script, it has now been adapted to support ROS2-based spawning workflows.
    *   This integration allows the use of launch files or services in ROS2 for simulation lifecycle management.

## ROS2 Integration

All models and scripts here have been prepared to support ROS2-native workflows. The `.sdf.jinja` templates and custom meshes are compatible with ROS2-driven spawning tools and enable seamless control, coordination, and observation of simulated UAVs via ROS2 nodes.

This allows researchers to perform simulation experiments without relying on shell scripts for spawning, enabling better integration with launch files and service-based orchestration.
