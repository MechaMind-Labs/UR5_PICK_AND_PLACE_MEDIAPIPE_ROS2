"""
slider_controller.launch.py
----------------------------
Full launch: Gazebo + controllers + GUI sliders + gripper mimic sync.

Node roles
──────────
  joint_state_broadcaster      — publishes /joint_states from Ignition physics
  arm_controller               — JTC for 6 UR5 arm joints
  gripper_controller           — JTC for finger_joint (the one actuated DOF)
  gripper_mimic_controller     — JTC for the 5 mimic joints (physics hold)
  joint_state_publisher_gui    — slider GUI → publishes to /joint_commands
  slider_controller node       — /joint_commands → arm + gripper JointTrajectory
  gripper_mimic_controller node— /joint_states finger_joint → mimic JointTrajectory

Sequencing
──────────
  t=0s  Gazebo starts, ign_ros2_control plugin loads
  t=8s  JSB spawner (waits up to 30s for controller_manager)
  t=JSB arm + gripper + gripper_mimic spawners (OnProcessExit)
  t=8s  gripper_mimic_controller NODE starts (needs /joint_states, t>=8s fine)
  t=14s GUI sliders + slider_controller node start
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ur5_description_pkg = get_package_share_directory("ur5_description")
    ur5_controller_pkg  = get_package_share_directory("ur5_controller")

    # ── 1. Gazebo ──────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur5_description_pkg, "launch", "gazebo.launch.py")
        )
    )

    # ── 2a. joint_state_broadcaster ───────────────────────────────────────────
    jsb_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        output="screen",
    )
    delayed_jsb = TimerAction(period=8.0, actions=[jsb_spawner])

    # ── 2b. arm + gripper + mimic — after JSB exits ───────────────────────────
    arm_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["arm_controller",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        output="screen",
    )
    gripper_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["gripper_controller",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        output="screen",
    )
    gripper_mimic_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["gripper_mimic_controller",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        output="screen",
    )
    spawn_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[arm_spawner, gripper_spawner, gripper_mimic_spawner],
        )
    )

    # ── 3. gripper_mimic_controller NODE ──────────────────────────────────────
    #    Reads /joint_states → computes mimic positions → publishes to
    #    /gripper_mimic_controller/joint_trajectory.
    #    Starts at t=8s (same as JSB) — it will wait for /joint_states naturally.
    gripper_mimic_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="ur5_controller",
                executable="gripper_mimic_controller",
                name="gripper_mimic_controller_node",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    # ── 4. GUI sliders ────────────────────────────────────────────────────────
    joint_state_publisher_gui = TimerAction(
        period=14.0,
        actions=[
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                parameters=[{"use_sim_time": True, "rate": 50}],
                remappings=[("/joint_states", "/joint_commands")],
                output="screen",
            )
        ],
    )

    # ── 5. slider_controller node ─────────────────────────────────────────────
    slider_node = TimerAction(
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
        gripper_mimic_node,
        joint_state_publisher_gui,
        slider_node,
    ])
