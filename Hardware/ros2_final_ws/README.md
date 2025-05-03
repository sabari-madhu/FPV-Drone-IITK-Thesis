# Final Hardware Implementation – Autonomous Vision-Based Drone Control

This package contains the final tested codebase used for the hardware implementation of the thesis, focusing on autonomous drone control in GPS-denied environments using visual detections and velocity-based motion planning. The system utilizes ArduPilot (via DDS bridge), ROS 2, and a YOLOv8 segmentation model to detect and track a person and respond with controlled drone movements.

## 🧭 System Overview

The system operates in two main components:

*   **Detection Node (`detection_node.py`):** Detects a person in the camera feed using YOLOv8 segmentation and publishes movement commands.
*   **Control Node (`ardu_master_node.py`):** Interprets and executes velocity commands, manages drone flight through PID-controlled altitude stabilization, and handles ArduPilot mode synchronization.

A launch file (`master.launch.py`) is provided to orchestrate the entire system.

## 📁 File Structure

| File                   | Description                                                                                                                                                               |
| ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `detection_node.py`    | Vision-based detection node using YOLOv8 to identify a person and generate movement commands (forward & yaw)                                                                  |
| `ardu_master_node.py`  | Master control node to interpret movement commands, manage drone velocity, stabilize height using PID, and coordinate with ArduPilot                                            |
| `master.launch.py`     | ROS 2 launch file to start the entire stack, initializing the detection node immediately and the master node with a delay                                                   |
| `yolo11n-seg.pt`       | Pretrained YOLOv8 segmentation model used for detecting persons (ensure this is present in the root directory)                                                              |

## 🚀 Launch Instructions

```bash
ros2 launch ardupilot_dds_tests master.launch.py
```

This:

1.  Starts the `detection_node` which begins processing `/image_raw` to detect people.
2.  Delays the start of `ardu_master_node` by 10 seconds to ensure vision is ready.
3.  Handles synchronization with ArduPilot (via `/ap/mode_switch`, `/ap/cmd_vel`, `/ap/pose/filtered`).

## 🧩 Key Functionalities

### 🧠 `detection_node.py`

*   Subscribes to `/image_raw` and processes frames via `yolo11n-seg.pt`.
*   On successful detection:
    *   Computes area coverage to infer distance.
    *   Computes left-right score for yaw alignment.
    *   Publishes a string command to `/movement` in format: `linear_<x>_yaw_<angle>`.
*   Only publishes commands if:
    *   The drone is in `GUIDED` mode.
    *   The previous motion is marked complete.

### ✈️ `ardu_master_node.py`

*   Subscribes to `/movement`, `/ap/pose/filtered`, and monitors flight state.
*   Controls drone motion via `/ap/cmd_vel` using:
    *   PID-controlled altitude stabilization.
    *   Dynamic yaw and forward motion execution.
*   Regularly checks ArduPilot's current mode via `/ap/mode_switch` and only acts in `GUIDED` mode.
*   Publishes control flags:
    *   `/guided_mode_activation` (Bool)
    *   `/movement_completion` (Bool)

### 🔁 PID Height Stabilization

*   Continuously corrects vertical drift using a PID controller:
    *   Default: `Kp=2.5`, `Ki=0.0`, `Kd=0.75`, `Max Output=4.0`
*   These values can be tuned via the launch file.

## 🛠️ Customization via Launch Arguments

You can override parameters during launch:

```bash
ros2 launch <your_package_name> master.launch.py \
  alt_pid_kp:=3.0 \
  yaw_velocity:=0.8 \
  max_angle_rotation:=20.0
```

## 🔄 Topic Communication Flow

```
    CameraFeed[/image_raw/] --> DetectionNode
    DetectionNode -->|String: "linear_x_yaw_y"| /movement
    DetectionNode -->|Bool| /movement_completion
    ArduMaster -->|TwistStamped| /ap/cmd_vel
    ArduMaster -->|String| /drone_mode
    ArduMaster -->|Bool| /guided_mode_activation
    ArduMaster --> /ap/mode_switch
    ArduPilot -->|PoseStamped| /ap/pose/filtered
```

## ⚠️ Notes

*   Make sure the flight controller (e.g., Cube Orange+) is in `GUIDED` mode for movement to activate.
*   This system assumes forward movement is in the X-axis direction relative to the drone frame.
*   Requires proper calibration of `/ap/pose/filtered` and camera alignment for reliable functioning.
*   The package my_copter_interfaces demonstrates how to define and use a custom action interface in ROS 2.

## 📦 Dependencies

*   ROS 2 Humble
*   ArduPilot DDS Bridge
*   Python packages: `ultralytics`, `cv_bridge`, `opencv-python`, `sensor_msgs`, `geometry_msgs`, `std_msgs`

## 🧪 Tested Setup

*   **Hardware:** Cube Orange+, Companion Computer (Raspberry Pi 4B), Camera Module
*   **Software Stack:** ROS 2 Foxy, ArduPilot 4.5+, YOLOv8 (segmentation model), DDS-based MAVROS alternative