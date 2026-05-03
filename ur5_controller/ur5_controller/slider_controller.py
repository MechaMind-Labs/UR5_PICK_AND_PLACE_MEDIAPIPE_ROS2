#!/usr/bin/env python3
"""
slider_controller.py
--------------------
Bridges /joint_commands (from joint_state_publisher_gui) to the
arm_controller and gripper_controller JointTrajectory topics.

Key design points
─────────────────
• Looks up joints BY NAME from the JointState — order-independent.
• Sets header.stamp so ign_ros2_control's trajectory controller accepts it.
• Uses a 200 ms time_from_start so the controller can physically track the
  slider even if Gazebo physics is running slower than real-time.
• Sends velocities=0 so the controller knows we want a hold at the target.
"""

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
GRIPPER_JOINTS = ["finger_joint"]

ARM_HOME     = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
GRIPPER_HOME = [0.0]


class SliderControl(Node):
    def __init__(self):
        super().__init__("slider_control")

        self.arm_pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 10)

        self.sub = self.create_subscription(
            JointState, "/joint_commands", self._cb, 10)

        self.get_logger().info("SliderControl ready — listening on /joint_commands")

    def _cb(self, msg: JointState):
        # Build name→position map; ignore joints we don't control
        pos = dict(zip(msg.name, msg.position))

        arm_positions = [pos.get(j, ARM_HOME[i])     for i, j in enumerate(ARM_JOINTS)]
        grp_positions = [pos.get(j, GRIPPER_HOME[i]) for i, j in enumerate(GRIPPER_JOINTS)]

        self._publish(self.arm_pub,     ARM_JOINTS,     arm_positions)
        self._publish(self.gripper_pub, GRIPPER_JOINTS, grp_positions)

    def _publish(self, pub, joint_names, positions):
        traj = JointTrajectory()
        traj.joint_names = joint_names

        # Stamp is required by some versions of ign_ros2_control's JTC
        traj.header.stamp = self.get_clock().now().to_msg()

        pt = JointTrajectoryPoint()
        pt.positions  = list(positions)
        pt.velocities = [0.0] * len(positions)
        pt.accelerations = [0.0] * len(positions)
        # 200 ms gives the physics engine time to reach the target.
        # Too short → controller rejects as "in the past".
        # Too long  → motion looks laggy.
        pt.time_from_start = Duration(sec=0, nanosec=200_000_000)

        traj.points = [pt]
        pub.publish(traj)


def main():
    rclpy.init()
    node = SliderControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
