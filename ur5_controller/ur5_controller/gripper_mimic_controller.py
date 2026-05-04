#!/usr/bin/env python3
"""
gripper_mimic_controller.py
----------------------------
Ignition Gazebo does NOT simulate URDF <mimic> joints — every revolute joint
is an independent physics body. Without this node all 5 mimic joints of the
Robotiq 2F-85 are free bodies under gravity and fly apart instantly.

This node:
  1. Subscribes to /joint_states (published by joint_state_broadcaster)
  2. Reads the current position of finger_joint
  3. Computes each mimic joint's target:
       position = finger_joint_pos * multiplier + offset
  4. Publishes a JointTrajectory to /gripper_mimic_controller/joint_trajectory
     which drives all 5 mimic joints in Ignition physics to match finger_joint.

robot_state_publisher still reads the URDF <mimic> tags directly and computes
correct TF for RViz — this node only handles the Gazebo physics side.

Mimic relationships (from URDF):
  left_inner_knuckle_joint  = finger_joint × +1
  left_inner_finger_joint   = finger_joint × −1
  right_inner_knuckle_joint = finger_joint × −1   (axis flipped, so same effect)
  right_inner_finger_joint  = finger_joint × +1   (axis flipped)
  right_outer_knuckle_joint = finger_joint × −1   (axis flipped)
"""

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# (joint_name, multiplier)
# multiplier matches the <mimic> tag in the URDF
MIMIC_JOINTS = [
    ("left_inner_knuckle_joint",  +1.0),
    ("left_inner_finger_joint",   -1.0),
    ("right_inner_knuckle_joint", -1.0),
    ("right_inner_finger_joint",  +1.0),
    ("right_outer_knuckle_joint", -1.0),
]

FINGER_JOINT = "finger_joint"
DEADBAND     = 0.0005   # rad — ignore noise smaller than this
TIME_FROM_START_NS = 200_000_000  # 200 ms


class GripperMimicController(Node):
    def __init__(self):
        super().__init__("gripper_mimic_controller")

        self.pub = self.create_publisher(
            JointTrajectory,
            "/gripper_mimic_controller/joint_trajectory",
            10,
        )

        self.sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._cb,
            10,
        )

        self._last_finger_pos = None
        self.get_logger().info("GripperMimicController ready — syncing mimic joints to Ignition physics")

    def _cb(self, msg: JointState):
        if FINGER_JOINT not in msg.name:
            return

        idx = msg.name.index(FINGER_JOINT)
        finger_pos = msg.position[idx]

        # Dead-band: don't spam if finger hasn't moved
        if self._last_finger_pos is not None:
            if abs(finger_pos - self._last_finger_pos) < DEADBAND:
                return
        self._last_finger_pos = finger_pos

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = [j for j, _ in MIMIC_JOINTS]

        pt = JointTrajectoryPoint()
        pt.positions     = [finger_pos * mult for _, mult in MIMIC_JOINTS]
        pt.velocities    = [0.0] * len(MIMIC_JOINTS)
        pt.accelerations = [0.0] * len(MIMIC_JOINTS)
        pt.time_from_start = Duration(sec=0, nanosec=TIME_FROM_START_NS)

        traj.points = [pt]
        self.pub.publish(traj)


def main():
    rclpy.init()
    node = GripperMimicController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
