"""
slider_controller.launch.py
---------------------------
Launches Gazebo + controllers + GUI sliders for interactive UR5 control.

Sequencing:
  t=0s   → Gazebo starts, ign_ros2_control plugin loads inside sim
  t=8s   → joint_state_broadcaster spawned (waits up to 30s for CM)
  t=JSB  → arm_controller + gripper_controller spawned (OnProcessExit)
  t=14s  → joint_state_publisher_gui + slider_controller start
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ur5_description_pkg = get_package_share_directory("ur5_description")
    ur5_controller_pkg  = get_package_share_directory("ur5_controller")

    # ── 1. Gazebo ─────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur5_description_pkg, "launch", "gazebo.launch.py")
        )
    )

    # ── 2a. joint_state_broadcaster ───────────────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    delayed_jsb = TimerAction(period=8.0, actions=[joint_state_broadcaster_spawner])

    # ── 2b. arm + gripper spawned after JSB exits cleanly ─────────────────────
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    spawn_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner],
        )
    )

    # ── 3. joint_state_publisher_gui ──────────────────────────────────────────
    # IMPORTANT: do NOT pass source_list as a parameter — an empty list becomes
    # a tuple () which crashes the launch system with:
    #   "Expected 'value' to be one of [float, int, str, bool, bytes], got tuple"
    #
    # The GUI reads /robot_description automatically — no source_list needed.
    # Remapped so it publishes to /joint_commands instead of /joint_states,
    # keeping it separate from the real /joint_states from joint_state_broadcaster.
    joint_state_publisher_gui = TimerAction(
        period=14.0,
        actions=[
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                parameters=[{
                    "use_sim_time": True,
                    "rate": 50,
                }],
                remappings=[
                    ("/joint_states", "/joint_commands"),
                ],
                output="screen",
            )
        ],
    )

    # ── 4. slider_controller ──────────────────────────────────────────────────
    slider_control_node = TimerAction(
        period=14.0,
        actions=[
            Node(
                package="ur5_controller",
                executable="slider_controller",
                name="slider_control",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    return LaunchDescription([
        gazebo,
        delayed_jsb,
        spawn_after_jsb,
        joint_state_publisher_gui,
        slider_control_node,
    ])
