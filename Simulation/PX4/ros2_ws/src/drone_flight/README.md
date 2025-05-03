# Drone Offboard Control and Pattern Detection (ROS2 + PX4)

This folder contains multiple ROS2 nodes designed to work with PX4-based drones in **Offboard mode**. The system is tested in simulation and supports autonomous movement and vision-based pattern detection.

---

## Pattern Detection Node

**File**: `pattern_detection_node.py`  
**Purpose**: Detects a predefined pattern (template) using:
- Blue color segmentation
- SIFT-based feature matching
- Multi-scale template matching  
**Publishes**: Pattern detection confidence to `/pattern_score`

**Input Requirement**:
- A sample pattern image named `pattern.png` placed in the root directory.

**Visual Example**:
![pattern](pattern.png)

---

## Pattern-Aware Offboard Controller

**File**: `offboard_pattern_controller.py`  
**Purpose**: Controls drone movement using `/pattern_score`:
- Takes off to a fixed altitude
- Rotates to search for pattern
- Moves forward when detected
- Hovers when confidence is high

**Depends On**:
- Pattern detection node (`pattern_detection_node.py`)

---

## Square Path Offboard Controller

**File**: `offboard_square_path.py`  
**Purpose**: Moves drone in a **square path**:
- Arms and takes off
- Follows 4 sides of a square with fixed speed and yaw
- Returns to initial point and lands

**Standalone**: Does not need pattern detection.

---

## Rotational Test Controller

**File**: `offboard_rotation_test.py`  
**Purpose**:
- Takes off and rotates the drone by 90° (π/2 radians)
- Demonstrates yaw-speed control in offboard mode

**Standalone**: Does not need pattern detection.

---

## Circular Path Controller

**Embedded Function**: In `offboard_control_takeoff_and_land.py` (via `timer_callback1`)  
**Purpose**:
- Makes drone move in a circular trajectory at a fixed radius
- Can be activated by switching the timer callback

---

## Notes

- All nodes are ROS2 Python nodes compatible with PX4 offboard mode.
- Ensure DDS bridges are properly set up before launching nodes.
- Use `colcon build` and `source install/setup.bash` before running.

---

## Dependencies

- ROS 2 Foxy
- PX4 Firmware and simulation setup
- Python OpenCV (`cv2`), `numpy`, and `cv_bridge`
