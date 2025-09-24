import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'zenorak_ros2_control'

    # -----------------------------
    # Robot State Publisher (URDF)
    # -----------------------------
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ])
    )

    # -----------------------------
    # ros2_control with controllers
    # -----------------------------
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'controller.yaml'
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description}, controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # -----------------------------
    # Joystick teleop
    # -----------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    joy_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'joystick.yaml'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params, {'use_sim_time': use_sim_time}],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/diff_cont/cmd_vel_unstamped')]
    )

    # Optional: add header stamp to twist messages
    # twist_stamper = Node(
    #     package='twist_stamper',
    #     executable='twist_stamper',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     remappings=[('/cmd_vel_in','/diff_cont/cmd_vel_unstamped'),
    #                 ('/cmd_vel_out','/diff_cont/cmd_vel')]
    # )

    # -----------------------------
    # Launch all
    # -----------------------------
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        rsp,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        joy_node,
        teleop_node,
        # twist_stamper
    ])
