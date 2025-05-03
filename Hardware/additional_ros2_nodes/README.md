# Final Hardware Implementation: Test Nodes

This folder contains various ROS2 test nodes developed as part of the final hardware implementation in the thesis. Each node plays a distinct role in evaluating functionalities such as takeoff, vision-based tracking, detection-based movement control, and height-stabilized square path navigation.

## Node Descriptions

### 1. `cam_tracking.py` – YOLO + DeepSORT-based Person Tracker

Subscribes to `/image_raw`, detects people using YOLOv8, and tracks them with DeepSORT.

Publishes `annotated_image` with bounding boxes and computed metrics:

*   **Movement Score:** Variation in bounding box area across frames.
*   **Left-Right Score:** Person's horizontal displacement.

Useful for real-time person-following logic validation.

### 2. `camera_testing_node.py` – YOLO Segmentation & Horizontal Scoring

Uses `yolo11n-seg.pt` for person segmentation and computes a left-right score based on the first detected person.

Publishes:

*   `/annotated_images`: Frame with bounding box and segmentation overlay.
*   `/person_horizontal_score`: Float value indicating horizontal offset.

Used to test accuracy and consistency of person detection and scoring.

### 3. `detection_node_test.py` – Boolean-Based Movement Decision Node

Receives:

*   `/detection_input` (Bool): Detection flag.
*   `/guided_mode_activation` (Bool): System state validation.
*   `/movement_completion` (Bool): Ensures one move finishes before another starts.

Publishes `/movement` (String) commands like:

*   `linear_2.0_yaw_-15.0` when detection is true.
*   `linear_0.0_yaw_-30.0` when detection is false.

Demonstrates rule-based motion control logic.

### 4. `mavlink_takeoff_node.py` – Direct MAVLink Takeoff Node

Uses `pymavlink` to perform:

*   `GUIDED` mode activation.
*   Arming the vehicle.
*   Commanding takeoff to a specified altitude.

Publishes `takeoff_complete` (Bool) upon success/failure.

Can be used in non-ROS2 MAVLink setups or where direct control is required.

### 5. `square1.py` – Square Path with Simple Altitude Stabilization

Runs square trajectory using velocity commands.

Attempts to stabilize altitude using basic linear error correction.

Interfaces with:

*   `/ap/cmd_vel`
*   `/ap/pose/filtered`
*   `/ap/geopose/filtered`
*   Arming and mode switch services.

Used to validate overall flight path control and message flow under ArduPilot.

### 6. `square2_height_stabilization.py` – Square Path with PID-Controlled Altitude

Enhanced version of `square1.py` with:

*   PID-based altitude stabilization.
*   State machine for `FORWARD` and `ROTATE` modes.
*   Real-time mode querying.

Ensures tight control over flight trajectory and altitude.

## Usage Notes

*   All scripts are written in Python 3 and assume ROS2 and ArduPilot-based integration.
*   For visual nodes (`cam_tracking.py`, `camera_testing_node.py`), ensure camera topics are available.
*   For control nodes (`square1.py`, `square2_height_stabilization.py`), verify drone is connected, armed, and in `GUIDED` mode.

## Dependencies

*   ROS2 Humble
*   ArduPilot Micro ROS interface (for `/ap/*` topics).
*   Python packages:
    *   `ultralytics`
    *   `cv_bridge`
    *   `deep_sort_realtime`
    *   `pymavlink`
    *   `OpenCV`
    *   `numpy`