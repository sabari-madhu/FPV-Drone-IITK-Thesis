from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # First node to start
    square_node = Node(
        package='ardupilot_dds_tests',
        executable='square',
        name='square_node'
    )

    # Second node to start after a delay
    camera_node = Node(
        package='ardupilot_dds_tests',
        executable='camera',
        name='camera_node'
    )

    # Wrap the camera node in a timer action to delay its start
    delayed_camera = TimerAction(
        period=2.0,  # 2 second delay
        actions=[camera_node]
    )

    return LaunchDescription([
        square_node,
        delayed_camera
    ])