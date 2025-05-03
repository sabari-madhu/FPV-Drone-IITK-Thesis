# FPV-Drone-IITK-Thesis

# 🛸 Target-Based Visual Navigation in GPS-Denied Environments

**Autonomous Drone Control using Vision and Velocity Commands**

This repository contains the full simulation and hardware codebase for my work on "Target-Based Visual Odometry for Autonomous Drone Navigation in GPS-Denied Environments". The system enables a drone to visually detect, track, and follow a human target using onboard computer vision and navigate using velocity commands—without relying on GPS.

## 🔍 Project Objective

To design and implement an autonomous drone control system capable of:

*   Operating in GPS-denied indoor environments
*   Visually detecting and following a moving human target
*   Using velocity-based offboard control via ROS 2 and ArduPilot
*   Running in both simulation (PX4/Gazebo) and hardware (Cube Orange+ / Raspberry Pi 4B) setups


## 🧪 Key Components

### ✅ Vision-Based Detection

*   Uses YOLOv8 segmentation (`yolo11n-seg.pt`) to detect persons in the camera feed
*   Computes left-right displacement and bounding box area to infer motion direction and distance

### ✅ Velocity-Based Drone Control

*   Uses ROS 2 to publish velocity commands to `/ap/cmd_vel`
*   Includes altitude stabilization via PID controller
*   Supports target tracking, square patterns, and decision logic based on detection confidence

### ✅ Mode Synchronization

*   Uses `/ap/mode_switch` to check and control ArduPilot mode
*   Motion logic only activates in `GUIDED` mode
*   Publishes internal flags for modular control (`/guided_mode_activation`, `/movement_completion`)

## 🧠 Simulation and Hardware Support

| Environment           | Stack                      | Features                                                                      |
| --------------------- | -------------------------- | ----------------------------------------------------------------------------- |
| PX4 Simulation        | Gazebo + PX4 SITL + ROS 2 | Multi-drone formation, controller test, detection verification                |
| ArduPilot Simulation  | DDS + ArduPilot + ROS 2  | Pattern following, guided-mode tests, control testing                         |
| Hardware Setup        | Cube Orange+, Pi 4B, USB camera      | Live YOLO-based detection + drone tracking & velocity control                |

## 🚀 Getting Started

ROS 2 Humble is recommended for harware.

1.  **Install dependencies**

    ```bash
    sudo apt install ros-foxy-cv-bridge python3-opencv
    pip install ultralytics opencv-python numpy
    ```

2.  **Launch the final implementation (hardware)**

    ```bash
    ros2 launch ardupilot_dds_tests master.launch.py
    ```

## 📚 Notable Highlights

✅ Complete end-to-end drone control via visual cues

✅ GPS-independent—relies purely on onboard visual information

✅ Modular ROS 2 nodes for scalability and reusability

✅ Code for both simulation environments and real flight tests

✅ PID control logic and multi-node coordination built from scratch