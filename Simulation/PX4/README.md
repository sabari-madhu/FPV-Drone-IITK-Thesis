### PX4-based Multi-Drone Offboard Control Framework

This repository contains a comprehensive PX4-based drone simulation environment built using **ROS2 (Foxy)** and **Gazebo Classic**, designed for autonomous drone experiments.

The system supports **multi-drone control**, **pattern-aware navigation**, and **sensor-integrated offboard control**, all deployed through modular and scalable ROS2 nodes.

---

## 📦 Package Overview

### 1. `controller/`

Provides a **drone spawner node** (`spawn_drone`) to dynamically launch multiple PX4 SITL drone instances in Gazebo, each with unique namespaces and ports.

#### Key Features:

- Jinja2-based dynamic `.sdf` file generation
- PX4 SITL process management with logging
- Namespaced drone topics (`/iris_1`, `/iris_2`, ...)
- Compatible with standard PX4 models (`iris`, `rover`, `plane`, etc.)

#### Usage:

```bash
ros2 run controller drone_spawner --ros-args -p model:=iris -p num_instances:=3
```

### 2. `sample_code_package/`

Contains various offboard control nodes supporting both single and multi-drone scenarios, with and without sensor feedback.

#### Core Nodes:

| Node                                         | Description                                                       |
| -------------------------------------------- | ----------------------------------------------------------------- |
| `offboard_control_sample1.py`                | Simple arm-takeoff-hover-land loop                                |
| `offboard_control_sample2.py`                | Enhanced version with better QoS                                  |
| `offboard_control_twist_sample.py`            | Converts TwistStamped to PX4 trajectory commands                    |
| `offboard_control_twist_msg_based_sample.py` | Enables message-driven velocity control                         |
| `offboard_control_multi_drones_sample.py`    | Multi-drone synchronized flight logic                             |
| `offboard_control_multi_drones_read_sensors_sample.py` | Integrates LIDAR for obstacle-aware control                     |
| `offboard_control_multi_drones_rplidar_sample.py` | RPLIDAR-based obstacle avoidance and mapping                  |
| `processes.py`                               | Utility node to manage PX4/Gazebo subprocesses                     |

#### Launch:

`multi_drone_spawn_and_run.launch.py`: Spawns drones and initializes corresponding control nodes.

### 3. Pattern-Based Control & Detection (standalone)

These nodes implement template-based visual navigation, where movement is driven by pattern recognition confidence.

### 3. Custom Gazebo Components

This directory also has custom gazebo components which has been created for our simulation.

#### Nodes:

| Node                         | Description                                                 |
| ---------------------------- | ----------------------------------------------------------- |
| `pattern_detection_node.py`  | Detects patterns using color segmentation + SIFT          |
| `offboard_pattern_controller.py` | Moves toward detected pattern based on confidence       |
| `offboard_square_path.py`      | Executes square path in offboard mode                      |
| `offboard_rotation_test.py`    | Demonstrates controlled yaw rotation                       |
| `offboard_control_takeoff_and_land.py` | Contains embedded logic for circular motion           |

#### Input:

Requires `pattern.png` in the root directory for detection.

## 🧭 Simulation Architecture

- **Offboard Mode:** All movement is controlled via trajectory setpoints or Twist messages.
- **Multi-Drone Isolation:** Each drone has a unique namespace (`/drone1`, `/drone2`, etc.) and PX4 instance.
- **Sensor Feedback:** Optional LIDAR integration for obstacle awareness.
- **Visual Input:** Uses OpenCV + SIFT for detecting fixed visual patterns.
- **Process Control:** External process handler ensures PX4/Gazebo launch consistency.

## ✅ Prerequisites

- PX4-Autopilot (SITL) built with Gazebo Classic
- ROS2 Foxy
- Python deps: `jinja2`, `cv2`, `numpy`, `cv_bridge`
- Gazebo model templates located in PX4 sim model paths

## 🚀 Quickstart

**Build:**

```bash
colcon build --packages-select controller sample_code_package
source install/setup.bash
```

**Launch Simulation:**

```bash
ros2 launch sample_code_package multi_drone_spawn_and_run.launch.py num_drones:=3 model_name:=iris
```

**Run Pattern Detection (if needed):**

Ensure `pattern.png` is available, then launch:

```bash
ros2 run <your_package> pattern_detection_node
ros2 run <your_package> offboard_pattern_controller
```

## 📂 Folder Structure (Example)

```
.
├── controller/
│   └── spawn_drone.py
├── sample_code_package/
│   ├── offboard_control_sample1.py
│   ├── offboard_control_multi_drones_sample.py
│   └── ...
├── drone_flight/
│   ├── pattern_detection_node.py
│   ├── offboard_pattern_controller.py
│   ├── multi_drone_spawn_and_run.launch.py
│   ├── offboard_pattern_controller.py
|
└── pattern.png
```

## 🔧 Notes

- Pattern-based navigation is an optional behavior layer.
- You can combine multi-drone launch with specific control nodes per drone.
- Adjust `node_id` and `namespace` in multi-drone scripts to target individual agents.
- For circular motion, enable `timer_callback1` inside `offboard_control_takeoff_and_land.py`.