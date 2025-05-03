from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments for ArduMasterNode parameters
    alt_pid_kp_arg = DeclareLaunchArgument('alt_pid_kp', default_value='2.5')
    alt_pid_ki_arg = DeclareLaunchArgument('alt_pid_ki', default_value='0.0')
    alt_pid_kd_arg = DeclareLaunchArgument('alt_pid_kd', default_value='0.75')
    alt_pid_max_output_arg = DeclareLaunchArgument('alt_pid_max_output', default_value='4.0')
    linear_velocity_arg = DeclareLaunchArgument('linear_velocity', default_value='1.0')
    yaw_velocity_arg = DeclareLaunchArgument('yaw_velocity', default_value='1.0')
    # Declare launch argument for DetectionNode parameter
    max_angle_rotation_arg = DeclareLaunchArgument('max_angle_rotation', default_value='60.0')

    # Launch the DetectionNode immediately
    detection_node = Node(
        package='ardupilot_dds_tests',
        executable='detection',
        name='detection',
        output='screen',
        parameters=[{
            'max_angle_rotation': LaunchConfiguration('max_angle_rotation'),
        }]
    )

    # Define the ArduMasterNode launch (delayed by 10 seconds)
    ardu_master_node = Node(
        package='ardupilot_dds_tests',
        executable='ardu_master',
        name='ardu_master',
        output='screen',
        parameters=[{
            'alt_pid_kp': LaunchConfiguration('alt_pid_kp'),
            'alt_pid_ki': LaunchConfiguration('alt_pid_ki'),
            'alt_pid_kd': LaunchConfiguration('alt_pid_kd'),
            'alt_pid_max_output': LaunchConfiguration('alt_pid_max_output'),
            'linear_velocity': LaunchConfiguration('linear_velocity'),
            'yaw_velocity': LaunchConfiguration('yaw_velocity'),
        }]
    )

    # TimerAction to delay ArduMasterNode launch
    delayed_master = TimerAction(
        period=10.0,
        actions=[ardu_master_node]
    )

    return LaunchDescription([
        # Launch arguments
        alt_pid_kp_arg,
        alt_pid_ki_arg,
        alt_pid_kd_arg,
        alt_pid_max_output_arg,
        linear_velocity_arg,
        yaw_velocity_arg,
        max_angle_rotation_arg,
        # Nodes
        detection_node,
        delayed_master
    ])
