from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # Get package directory
    pkg_share = get_package_share_directory('ur5_description')
    
    # Path to URDF file WITH GRIPPER
    urdf_file = os.path.join(pkg_share, 'urdf', 'ur5_with_gripper.urdf.xacro')
    
    # Path to RViz config (optional, will create later)
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'display.rviz')
    
    # Process the URDF file with required arguments
    robot_description_content = Command([
        'xacro ', urdf_file,
        ' name:=ur5',
        ' ur_type:=ur5',
        ' tf_prefix:=""',
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )
    
    # Joint State Publisher GUI (to control robot and gripper)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
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