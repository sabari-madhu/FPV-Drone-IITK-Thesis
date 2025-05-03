import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    launch_description = launch.LaunchDescription()

    # === Configuration Options ===
    n_agents = 4  # Change number of drones to spawn
    model_name = "iris_rplidar_custom"  # Model used in Gazebo
    process_terminal = True  # If True, open 'processes' in a separate terminal
    control_executable = "drone_twist1"  # Options: 'multi_drone_custom1', 'drone_twist2', etc.

    # === Optional: Start a process launcher (e.g., to launch sensor/data fusion nodes) ===
    if process_terminal:
        processes = launch_ros.actions.Node(
            package='sample_code_package',
            namespace='sample_code_package',
            executable='processes',
            name='processes',
            prefix='gnome-terminal --'  # Or 'xterm -e' based on your system
        )
        launch_description.add_action(processes)
    else:
        processes = None

    # === Spawn drone models ===
    spawn_drone_action = launch_ros.actions.Node(
        package='controller',
        executable='spawn_drones',
        output='screen',
        name='spawn_drone',
        parameters=[{'num_instances': n_agents, "model": model_name}]
    )

    # Use OnProcessExit to wait for 'processes' to finish before spawning drones
    if processes:
        launch_description.add_action(
            launch.actions.RegisterEventHandler(
                OnProcessExit(
                    target_action=processes,
                    on_exit=[spawn_drone_action]
                )
            )
        )
    else:
        launch_description.add_action(spawn_drone_action)

    # === Launch drone control nodes after spawn ===
    for i in range(n_agents):
        node_id = i + 1
        drone_control_node = launch_ros.actions.Node(
            package='sample_code_package',
            executable=control_executable,
            output='screen',
            name=f'drone{node_id}',
            parameters=[{'node_id': node_id, 'num_nodes': n_agents, "model_name": model_name}]
        )

        launch_description.add_action(
            launch.actions.RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_drone_action,
                    on_exit=[drone_control_node]
                )
            )
        )

    return launch_description
