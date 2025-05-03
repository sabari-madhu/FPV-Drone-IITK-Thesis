### ROS2-based Drone Simulation using ArduPilot

This repository contains the complete ROS2 simulation setup.

It uses **ArduPilot**, **Gazebo**, and **ROS2** to demonstrate autonomous takeoff, motion planning, and person-following behavior in a GPS-denied setting through modular node-based architecture. The system leverages internal velocity control, visual detection, and decision fusion, enabling a drone to perform structured patterns and dynamically adapt based on target presence.

---

## 📦 Packages Overview

### 1. `ardupilot_dds_tests/`

This package focuses on foundational behaviors such as autonomous takeoff, geometric path execution, and vision-based detection.

#### Key Nodes:
| Node | Description |
|------|-------------|
| `takeoff.py` | Arms and commands drone to take off using DDS topics |
| `square_tragectory.py` | Executes square motion using `/ap/cmd_vel` |
| `image_process.py` | Performs head detection from image stream |
| `tracking_different_implementation.py` | Sends PID-inspired velocity commands to follow detected person |

#### Launch:
- `drone.launch.py`: Launches square path and detection nodes with time delay coordination.

---

### 2. `thesis_nodes/`

This package orchestrates a layered and intelligent behavior based on perception and internal command routing, forming the core of the thesis system.

#### Key Nodes:
| Node | Description |
|------|-------------|
| `ArdupilotConnector.py` | Interfaces ArduPilot and internal system for velocity and mode |
| `CameraProcessing.py` | Runs YOLOv8 model for real-time person detection |
| `ScanPerson.py` | Commands circular scan motion when no person is found |
| `Tracking.py` | Aligns drone with detected person using region and size heuristics |
| `MasterController.py` | Fuses scan and tracking control, prioritizes person-following |

#### Launch:
- `thesis_all.launch.py`: Launches the full perception-control pipeline.

---

## 🚁 Simulation Architecture

- **Control Loop**: Offboard velocity control using `/ap/cmd_vel`
- **Decision Logic**: Master node selects between scan or track commands
- **Mode Handling**: Drone remains operational only in `GUIDED` mode (via `/ap/mode_switch`)
- **Pose Monitoring**: Uses `/ap/pose/filtered` and `/ap/geopose/filtered` for altitude checks and tracking triggers
- **Perception**: YOLOv8/haar-cascade models detect person/face
- **Failover**: Scanning behavior acts as fallback when no detection is found

---

## 🧪 Key Features Demonstrated

- Autonomous takeoff using ArduPilot services
- Geometric path following (square trajectory)
- Visual perception-driven behavior (head/person detection)
- Dynamic switching between scanning and tracking
- Full simulation in Gazebo with modular ROS2 nodes

---

## ✅ Prerequisites

- ArduPilot with DDS bridge
- ROS2 Humble
- Gazebo simulator
- YOLOv8 weights (`yolo11n.pt`) placed appropriately for detection node
- Haar cascades for fallback face detection (auto-downloaded if not found)

---

## 📝 Notes

- All velocity and decision logic is internal — no manual Twist commands
- The system is fully autonomous once launched and requires no manual intervention post-takeoff
- Designed to be extended for swarm coordination or advanced perception integration

---

## 📂 Folder Structure (Sample)

```
├── ardupilot_dds_tests/
│ ├── takeoff.py
│ ├── square_tragectory.py
│ ├── image_process.py
│ └── tracking_different_implementation.py
│ └── launch/drone.launch.py
├── thesis_nodes/
│ ├── ArdupilotConnector.py
│ ├── CameraProcessing.py
│ ├── ScanPerson.py
│ ├── Tracking.py
│ ├── MasterController.py
│ └── launch/thesis_all.launch.py
├── README.md (this file)
```