from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zenorak_teleop',
            executable='teleop_wasd',   # matches scripts/teleop_wasd.py
            name='teleop_py',
            output='screen',
            # Keyboard input typically WON'T work via launch, prefer ros2 run.
            # emulate_tty=True doesn't guarantee stdin forwarding.
        )
    ])
