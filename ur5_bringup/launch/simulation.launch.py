import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # ------------------- Gazebo -------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ur5_description"),
                "launch",
                "gazebo.launch.py"
            )
        )
    )

    # ------------------- Controllers -------------------
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ur5_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    # ------------------- Vision Nodes -------------------
    hand_detection_node = Node(
        package="ur5_vision",
        executable="hand_detection",
        name="hand_detection_node",
        output="screen"
    )

    hand_controller_node = Node(
        package="ur5_vision",
        executable="ur5_hand_controller",
        name="ur5_hand_controller_node",
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        controller,
        hand_detection_node,
        # hand_controller_node,
    ])