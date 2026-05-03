"""
slider_controller.launch.py
---------------------------
One-command launch that:
  1. Starts Gazebo with the UR5 spawned and ign_ros2_control running
  2. Spawns joint_state_broadcaster → arm_controller → gripper_controller
  3. Opens joint_state_publisher_gui (the sliders)
  4. Runs slider_controller node to forward slider values as JointTrajectory

Usage:
  ros2 launch ur5_controller slider_controller.launch.py
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

    # ── 2a. joint_state_broadcaster spawner (first — others depend on it) ─────
    #   Delayed 8 s to let Gazebo + ign_ros2_control fully start.
    #   Increase this if you see "controller_manager not available" errors.
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

    # ── 2b. arm + gripper spawned AFTER joint_state_broadcaster exits OK ──────
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

    # Wait for JSB to be active before loading the trajectory controllers
    spawn_arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner],
        )
    )

    # ── 3. joint_state_publisher_gui ─────────────────────────────────────────
    #   Remapped so it publishes on /joint_commands instead of /joint_states
    #   (avoids fighting with the real /joint_states from joint_state_broadcaster)
    #   Delayed until after controllers are loaded so the GUI sources
    #   /robot_description which is already published by then.
    joint_state_publisher_gui = TimerAction(
        period=12.0,
        actions=[
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                parameters=[{"use_sim_time": True}],
                remappings=[
                    ("/joint_states", "/joint_commands"),
                ],
                output="screen",
            )
        ],
    )

    # ── 4. slider_controller ─────────────────────────────────────────────────
    #   Reads /joint_commands → publishes to arm/gripper JointTrajectory topics
    slider_control_node = TimerAction(
        period=12.0,
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
        spawn_arm_after_jsb,
        joint_state_publisher_gui,
        slider_control_node,
    ])
