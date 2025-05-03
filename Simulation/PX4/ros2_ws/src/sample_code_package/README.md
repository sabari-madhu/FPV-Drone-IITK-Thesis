# sample_code_package

## Overview

`sample_code_package` provides ROS2 offboard-control nodes and launch scripts to automate drone simulations using PX4 SITL and Gazebo. Each node offers specific control strategies or sensor integrations and uses the companion `controller` package for basic flight operations.

## Launch File

### `multi_drone_spawn_and_run.launch.py`

**Purpose:** Automatically spawns multiple drones and initializes control nodes in Gazebo.

**Key Features:**

*   Accepts parameters `num_drones` and `model_name`.
*   Uses Gazebo services to spawn drone entities.
*   Launches separate control nodes for each drone.

## Nodes

### Single Drone Control

*   **`offboard_control_sample1.py`**:
    *   Arms, takes off to a preset altitude, hovers, and lands.

*   **`offboard_control_sample2.py`**:
    *   Improved QoS and command timing for smoother operation.

*   **`offboard_control_twist_sample.py`**:
    *   Converts velocity commands (`TwistStamped`) to PX4 trajectory setpoints, maintaining altitude.

*   **`offboard_control_twist_msg_based_sample.py`**:
    *   Initiates control based on custom messages, allowing pause and resume of velocity-based commands.

### Multi-Drone Control

*   **`offboard_control_multi_drones_sample.py`**:
    *   Controls multiple drones, differentiated using `node_id`. Manages synchronized takeoff sequences.

*   **`offboard_control_multi_drones_read_sensors_sample.py`**:
    *   Integrates LIDAR sensor feedback into multi-drone control to avoid obstacles.

*   **`offboard_control_multi_drones_rplidar_sample.py`**:
    *   Multi-drone setup using RPLIDAR data for real-time environmental mapping and obstacle avoidance (using RPLIDAR).

### Utility

*   **`processes.py`**:
    *   Manages external processes (PX4 SITL, Gazebo), monitors system logs, and handles process failures.

## Quick Start

### Setup:

1.  Build the packages:

    ```bash
    colcon build --packages-select controller sample_code_package
    ```

2.  Source the workspace:

    ```bash
    source install/setup.bash
    ```

### Launch Simulation:

```bash
ros2 launch sample_code_package multi_drone_spawn_and_run.launch.py num_drones:=3 model_name:=iris
```

### Monitor Topics:

```bash
ros2 topic echo /drone1/obstacle_distance
```

## Dependencies

*   ROS2 Foxy
*   PX4 SITL & Gazebo
*   `controller` package
*   ROS2 Messages: `geometry_msgs`, `sensor_msgs`, `px4_msgs`, `std_msgs`