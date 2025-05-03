# ardupilot_dds_tests

This ROS2 package provides a set of nodes for autonomous drone control and vision-based tracking using ArduPilot DDS middleware. It includes nodes for takeoff, executing square trajectories, visual detection using a camera, and a person-tracking control mechanism.

---

## Nodes

### 1. `takeoff.py`

**Function**: Initiates and monitors the drone takeoff sequence.

- Arms the drone using `/ap/arm_motors`
- Switches to `GUIDED` mode via `/ap/mode_switch`
- Performs takeoff via `/ap/experimental/takeoff`
- Monitors geopose and local pose data during ascent

---

### 2. `square_tragectory.py`

**Function**: Executes a square trajectory motion.

- Publishes velocity commands on `/ap/cmd_vel`
- Subscribes to `/ap/pose/filtered` and `/ap/geopose/filtered` for feedback
- Switches between `GUIDED` and `RTL` (Return to Launch) modes
- Demonstrates a full autonomous movement loop after takeoff

---

### 3. `image_process.py`

**Function**: Performs real-time human detection using a camera feed.

- Subscribes to `/camera/image`
- Splits the image into thirds (left, middle, right)
- Detects heads using OpenCV Haar cascades for frontal and profile faces
- Publishes a detection result string (`left,middle,right`) on `/human_detection/results`
- Applies fisheye distortion correction if required

---

### 4. `tracking_different_implementation.py`

**Function**: Sends motion commands to follow detected humans.

- Subscribes to `/internal/person_detection` (formatted as `region_offset_scale`)
- Publishes velocity commands to `/internal/cmd_vel_tracking`
- Uses PID-like behavior to rotate and move toward detected person
- Executes search behavior when no person is detected

---

## Launch File

### `drone.launch.py`

**Function**: Launches key nodes in the correct sequence.

- Launches:
  - `square` node (`square_tragectory.py`) immediately
  - `camera` node (`image_process.py`) after a 2-second delay
- Coordinates motion and vision tasks in a demo

---

## Summary Table

| Node                                 | Description                                           |
|--------------------------------------|-------------------------------------------------------|
| `takeoff.py`                         | Arms and commands the drone to take off              |
| `square_tragectory.py`               | Moves drone in a square pattern and returns to base  |
| `image_process.py`                   | Detects human presence using camera input            |
| `tracking_different_implementation.py` | Tracks and navigates towards detected individuals     |
| `drone.launch.py`                    | Launches square movement and camera nodes with delay |

---

## Notes

- Ensure camera topics and DDS setup match your system configuration.
- Haar cascades are downloaded automatically if not present locally.

