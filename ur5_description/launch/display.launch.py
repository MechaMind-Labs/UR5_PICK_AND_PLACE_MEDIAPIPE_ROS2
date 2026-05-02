from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Get package directory
    pkg_share = get_package_share_directory('ur5_description')

    # Path to URDF file WITH GRIPPER
    urdf_file = os.path.join(pkg_share, 'urdf', 'ur5_with_gripper.urdf.xacro')

    # Path to RViz config
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'display.rviz')

    # Process the URDF file with required arguments.
    # IMPORTANT: wrap in ParameterValue(..., value_type=str) so ROS2 does not
    # try to parse the URDF XML string as YAML (which always fails).
    robot_description_content = ParameterValue(
        Command([
            'xacro ', urdf_file,
            ' name:=ur5',
            ' ur_type:=ur5',
            ' tf_prefix:=',   # empty string — no quotes needed here
        ]),
        value_type=str,
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False,
        }],
    )

    # Joint State Publisher GUI
    # publish_default_positions lets the slider start at 0 for all joints,
    # including the single actuated gripper finger_joint.
    # The mimic joints (inner/outer knuckles) are driven by robot_state_publisher
    # via the URDF mimic tags — they do NOT need to appear in the GUI.
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'publish_default_positions': True,
            # Tell JSP to ignore mimic joints so they don't appear as
            # independent sliders (they are computed from finger_joint).
            'zeros': {},
        }],
    )

    # RViz Node
    rviz_args = ['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args,
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
    ])