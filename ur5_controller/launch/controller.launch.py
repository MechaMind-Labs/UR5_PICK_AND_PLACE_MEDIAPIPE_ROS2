from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation clock"
    )

    # ── 1. joint_state_broadcaster — must come first ──────────────────────────
    jsb_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        output="screen",
    )

    # ── 2. arm + gripper (actuated) — after JSB ───────────────────────────────
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

    # ── 3. gripper_mimic_controller — after JSB ───────────────────────────────
    #    Holds the 5 mimic joints in Ignition physics.
    #    Must be active before the sim physics can stabilise the gripper.
    gripper_mimic_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["gripper_mimic_controller",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        output="screen",
    )

    # Spawn all trajectory controllers only after JSB is active
    spawn_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[arm_spawner, gripper_spawner, gripper_mimic_spawner],
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        jsb_spawner,
        spawn_after_jsb,
    ])
