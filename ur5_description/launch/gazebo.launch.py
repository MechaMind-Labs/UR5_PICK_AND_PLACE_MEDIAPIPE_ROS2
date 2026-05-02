import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ur5_description = get_package_share_directory("ur5_description")

    # FIX 1: correct filename — ur5_with_gripper.urdf.xacro, not ur5.urdf.xacro
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(ur5_description, "urdf", "ur5_with_gripper.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="ur5")

    world_path = PathJoinSubstitution([
        ur5_description, "worlds",
        PythonExpression(["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
    ])

    model_path = str(Path(ur5_description).parent.resolve())
    model_path += pathsep + os.path.join(ur5_description, 'models')

    gazebo_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path)

    # FIX 2: removed is_ignition:= — no xacro file in this repo accepts that arg
    robot_description = ParameterValue(
        Command([
            "xacro ", LaunchConfiguration("model"),
            " ur_type:=ur5",
            " name:=ur5",
            " tf_prefix:=",
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
            "/gz_sim.launch.py"
        ]),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
        }.items()
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "ur5",
            "-x", "0.15", "-y", "0.6", "-z", "0.37",
            "-R", "0.0", "-P", "0.0", "-Y", "0.0",
        ],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # FIX 3: camera bridges removed — no camera in empty.world
            # add back when you have a camera sensor in the sim
        ],
    )

    return LaunchDescription([
        model_arg,
        world_name_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        # ros_gz_image_bridge removed — no camera sensor defined
    ])