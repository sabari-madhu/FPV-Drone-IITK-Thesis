# thesis_nodes

This ROS2 package is designed for drone control and person-following behavior using ArduPilot. It includes multiple nodes that handle command routing, person detection via a camera, autonomous scanning, person tracking, and a master controller that fuses decision logic.

---

## Nodes

### 1. `ArdupilotConnector.py`

**Node name**: `ardupilot_communicator`

**Function**:
- Interfaces with ArduPilot to:
  - Forward velocity commands from `/internal/cmd_vel` to `/ap/cmd_vel`
  - Provide a service for internal mode switching via `/internal/mode_switch`
  - Subscribe to `/ap/geopose/filtered` and republish on `/internal/geopose`

---

### 2. `CameraProcessing.py`

**Node name**: `camera_module`

**Function**:
- Uses a YOLOv8-based model to detect people from the `/camera/image` feed.
- Outputs detection messages (region, offset, scale) as strings to `/internal/person_detection`.

**Example message format**:  
`center_0.03_0.42` → region, horizontal offset, and relative size of detection.

---

### 3. `ScanPerson.py`

**Node name**: `scan`

**Function**:
- Publishes continuous circular motion commands to `/internal/cmd_vel_scan`.
- Used for exploratory motion when no person is detected.

---

### 4. `Tracking.py`

**Node name**: `tracking`

**Function**:
- Subscribes to `/internal/person_detection`.
- Publishes to `/internal/cmd_vel_tracking`.
- Tracks person using bounding box alignment:
  - If person is centered and growing, it moves forward.
  - If off-center, rotates to re-center.
  - If not detected, performs search behavior.

---

### 5. `MasterController.py`

**Node name**: `master_drone`

**Function**:
- Orchestrates motion control:
  - Waits until desired altitude is reached (via `/internal/geopose`)
  - Chooses between scan and tracking velocity commands
  - Publishes the final velocity command to `/internal/cmd_vel`

**Command selection**:
- If no detection: uses scan commands.
- If detection present: uses tracking commands.

---

## Launch File

### `thesis_all.launch.py`

**Function**: Launches all nodes for complete mission execution.

**Nodes Launched**:
- `ardupilot_communicator`
- `scan`
- `camera_module`
- `tracking`
- `master_drone`

---

## Summary Table

| Node                     | Description                                           |
|--------------------------|-------------------------------------------------------|
| `ardupilot_communicator` | Routes commands and handles mode switch              |
| `camera_module`          | Performs real-time person detection with YOLO        |
| `scan`                   | Moves in a circular pattern when person not found    |
| `tracking`               | Follows the detected person with offset correction   |
| `master_drone`           | Decision maker for selecting scan or tracking action |
| `thesis_all.launch.py`   | Launches full integrated mission                     |

---

## Notes

- Ensure `yolo11n.pt` is available in the expected path.
- Detection format must be `region_offset_scale` (e.g., `center_-0.02_0.35`).