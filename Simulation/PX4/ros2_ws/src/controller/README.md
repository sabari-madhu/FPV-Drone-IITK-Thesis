# Drone Spawner (ROS2 Controller Package)

A ROS2 package (`controller`) that provides a node called `spawn_drone` for automating the deployment of multiple PX4 SITL (Software-In-The-Loop) drone instances in Gazebo Classic. This package uses Jinja2 templating to dynamically generate SDF files and ensures each drone instance operates with proper namespace isolation and unique communication ports.

## Features

*   Dynamic SDF file generation using `.sdf.jinja` templates.
*   Automatic PX4 SITL instance launching with unique MAVLink and GStreamer ports.
*   Drone spawning via Gazebo's `/spawn_entity` service.
*   Clean ROS topic and service isolation through proper namespacing.

## Package Structure

```
controller/
├── spawn_drone.py  # Main ROS2 node for spawning drones
└── README.md         # Documentation
```

## Node: `spawn_drone`

### Parameters

| Name            | Type    | Default | Description                               |
|-----------------|---------|---------|-------------------------------------------|
| `model`         | string  | `iris`  | PX4 model to spawn                       |
| `num_instances` | integer | `2`     | Number of drone instances to spawn       |

## Supported Models

The following PX4 models are currently supported:

*   `iris`
*   `iris_custom`
*   `plane`
*   `standard_vtol`
*   `rover`
*   `r1_rover`
*   `typhoon_h480`

## Requirements

*   ROS 2 Foxy
*   PX4 Firmware (e.g., PX4-Autopilot) with SITL built
*   Gazebo Classic
*   Python dependencies:
    *   `jinja2` (`pip install jinja2`)

## Installation

1.  Clone this package into your ROS2 workspace:

2.  Build your workspace:

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select controller
    ```

3.  Source the workspace:

    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

## Usage

### Building PX4 SITL

Before using the drone spawner, ensure PX4 is properly built.

**Note:** Include `gazebo` in the `make` command to build the Gazebo simulator along with PX4.

### Running the Node

Launch the node with desired parameters:

```bash
ros2 run controller drone_spawner --ros-args -p model:=iris -p num_instances:=3
```

This will spawn 3 instances of the `iris` drone at spaced-out positions in the Gazebo environment. Adjust the `model` and `num_instances` parameters as needed. For example, to spawn a single `typhoon_h480` drone, use:

```bash
ros2 run controller drone_spawner --ros-args -p model:=typhoon_h480 -p num_instances:=1
```

## Operational Details

For each drone instance, the node:

1.  Validates the model name against supported types.
2.  Generates a unique `.sdf` file using the appropriate Jinja2 template.
3.  Starts a new PX4 SITL instance with a dedicated rootfs.
4.  Sends a spawn request to Gazebo with a proper namespace and offset position.

Each drone is uniquely identified with sequential naming:

`/iris_1`, `/iris_2`, `/iris_3`, etc.

With automatically assigned ports:

*   MAVLink TCP: 4561, 4562, ...
*   MAVLink UDP: 14561, 14562, ...
*   GStreamer video: 5601, 5602, ...

## Gazebo Integration

The node requires model templates located at:

`PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/<model>/<model>.sdf.jinja`

Generated SDF files are temporarily stored in `/tmp`. The script handles the specific location and naming.

## Logs

Each PX4 instance writes logs to:

```
<build_path>/rootfs/<instance_num>/out.log
<build_path>/rootfs/<instance_num>/err.log
```

Where `<build_path>` refers to your PX4 build directory. These logs are valuable for debugging individual PX4 instances.

## Shutdown

To terminate the simulation, use standard Gazebo or ROS2 shutdown procedures. All PX4 instances will be properly closed.

## Troubleshooting

*   **Model Not Found:** Ensure the specified `model` name matches the name of the `.sdf.jinja` file in the PX4 simulation directory.
*   **Gazebo Crashing:** Verify that Gazebo is correctly installed and configured with the PX4 environment. Check Gazebo's terminal output for error messages.
*   **Communication Issues:** Double-check the MAVLink and GStreamer ports being used and ensure they are not blocked by firewalls. Use `ros2 topic list` and `ros2 service list` to see the namespaced topics and services.
*   **SITL Build Issues:** Confirm that PX4 SITL has been built correctly. 
*   **Permission Issues:** Ensure you have the correct permissions to create files in `/tmp`, or adjust the temporary file directory if needed.
