# Autonomous Drone Simulation Framework (ROS2 + Gazebo + PX4/ArduPilot)

This repository presents a complete drone simulation environment.

It supports both **ArduPilot** and **PX4** systems, leveraging **ROS2**, **Gazebo Classic**, and onboard visual intelligence for autonomous UAV behavior without reliance on GPS.

---

## 🧭 Overview

The system enables:
- Fully autonomous drone takeoff and landing
- Pattern-based path planning (square, circular)
- Real-time human detection and tracking
- Visual pattern recognition and confidence-driven motion
- Multi-agent simulation and coordination
- Integration of LIDAR for obstacle avoidance
- Decision fusion based on visual perception and system state

---

## ✈️ ArduPilot-Based Architecture

The ArduPilot branch focuses on autonomous navigation through internal command routing and visual perception:

- Autonomous flight behaviors (e.g., takeoff, square paths)
- Haar-cascade and YOLO-based person detection
- PID-inspired visual tracking and motion control
- Decision fusion through a master controller that selects between exploration and tracking
- Internal topics for command isolation and abstraction over ArduPilot DDS interface

This stack demonstrates reactive behavior in GPS-denied conditions and is suitable for modular expansion into swarm-based or dynamic environments.

---

## 🚁 PX4-Based Architecture

The PX4 branch emphasizes scalable simulation and pattern-based offboard control:

- Dynamic multi-drone spawning with namespaced instances and isolated MAVLink ports
- Individual and multi-agent offboard controllers for synchronized flight
- Pattern-aware navigation using SIFT and color segmentation for template detection
- Integration with LIDAR and RPLIDAR for real-time sensor-aware motion planning
- Utilities to manage PX4 SITL processes and ensure robust operation

It supports both standalone and coordinated drone behaviors using a modular launch structure.

---

## 🖼️ Visual Intelligence

- **YOLOv8-based detection** is used for robust person detection in ArduPilot flows.
- **Color and SIFT-based pattern detection** is applied in PX4 environments for navigation triggers.
- **Haar cascades** provide backup for head detection when GPU inference is not available.

Visual feedback influences navigation decisions — either by initiating movement, stopping motion, or shifting into exploratory mode.

---

## 🧱 Simulation Environment

- Built with **ROS2 Foxy** and **Gazebo Classic**
- Uses DDS communication bridges for both PX4 and ArduPilot
- Real-time image processing for navigation in camera-driven workflows
- Clean abstraction layers between control, sensing, and behavior logic

---

## 🔧 Requirements

- PX4-Autopilot (SITL build with Gazebo)
- ArduPilot (DDS-enabled SITL)
- ROS2 Foxy
- Python packages: `jinja2`, `opencv-python`, `cv_bridge`, `numpy`
- Pretrained YOLOv8 weights for detection (where applicable)

---

## 🚀 Launch Workflows

- **ArduPilot System**: Launches full detection-control loop, suitable for person-following missions in simulation.
- **PX4 System**: Supports single and multi-drone spawning, offboard control, and pattern-aware navigation with optional sensor integration.

---

## 🧪 Key Demonstrations

- GPS-free navigation using visual features
- Autonomous flight planning with fallback behaviors
- Multi-agent drone coordination with isolation and robustness
- Seamless switching between exploration and pursuit behaviors