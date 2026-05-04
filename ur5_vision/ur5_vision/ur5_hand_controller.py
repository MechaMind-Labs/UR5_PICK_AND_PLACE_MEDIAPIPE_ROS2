#!/usr/bin/env python3
"""
ur5_hand_controller_node.py
============================
Node 2 - UR5 Joint Controller (subscribes to hand detection outputs)

Subscribes:
  /arm_joints       → std_msgs/Float32MultiArray  (6 joint angles in radians)
  /gripper_command  → std_msgs/Float32             (0=open, 1=close)

Publishes:
  /joint_trajectory_controller/joint_trajectory
                    → trajectory_msgs/JointTrajectory   (arm motion)
  /gripper_action_controller/... (via action client)

Also supports direct /follow_joint_trajectory action used by UR5+MoveIt2.

Design:
  - Applies joint limits to protect the robot
  - Publishes smooth incremental trajectories (not hard position jumps)
  - Gripper threshold: score > 0.6 → close, < 0.4 → open (hysteresis)
  - Status published to /hand_control_status for diagnostics

Author: MechaMind-Labs extension
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import numpy as np
import math
import time

from std_msgs.msg import Float32MultiArray, Float32, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory


# ---------------------------------------------------------------------------
# UR5 Joint names (as used by ros2_control / MoveIt2)
# ---------------------------------------------------------------------------
UR5_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# UR5 joint limits (radians) — conservative to stay safe
JOINT_LIMITS = [
    (-3.14, 3.14),    # shoulder_pan
    (-3.14, 0.0),     # shoulder_lift  (negative = arm up in UR convention)
    (-3.14, 3.14),    # elbow
    (-3.14, 3.14),    # wrist_1
    (-3.14, 3.14),    # wrist_2
    (-3.14, 3.14),    # wrist_3
]

# Safe home position (radians) — arm pointing up
HOME_JOINTS = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

# Gripper joint name (Robotiq 2F-85 as used in MechaMind-Labs repo)
GRIPPER_JOINT  = 'robotiq_85_left_knuckle_joint'
GRIPPER_OPEN   = 0.0      # radians
GRIPPER_CLOSED = 0.8      # radians (fully closed)

# Gripper controller action
GRIPPER_ACTION = '/gripper_action_controller/follow_joint_trajectory'
ARM_ACTION     = '/joint_trajectory_controller/follow_joint_trajectory'

# Also publish raw JointTrajectory (works without action, e.g. Gazebo direct)
ARM_TRAJ_TOPIC    = '/joint_trajectory_controller/joint_trajectory'
GRIPPER_TRAJ_TOPIC = '/gripper_controller/joint_trajectory'


# ---------------------------------------------------------------------------
class UR5HandControllerNode(Node):

    def __init__(self):
        super().__init__('ur5_hand_controller_node')

        # ---- Parameters ----
        self.declare_parameter('trajectory_duration_sec', 0.5)
        self.declare_parameter('gripper_close_threshold', 0.6)
        self.declare_parameter('gripper_open_threshold',  0.4)
        self.declare_parameter('use_action_client',       True)
        self.declare_parameter('max_joint_delta',         0.15)   # rad per cycle, speed limit
        self.declare_parameter('deadzone',                0.02)   # rad, ignore tiny changes
        self.declare_parameter('control_rate_hz',         20.0)

        self.traj_dur  = self.get_parameter('trajectory_duration_sec').value
        self.g_close   = self.get_parameter('gripper_close_threshold').value
        self.g_open    = self.get_parameter('gripper_open_threshold').value
        self.use_ac    = self.get_parameter('use_action_client').value
        self.max_delta = self.get_parameter('max_joint_delta').value
        self.deadzone  = self.get_parameter('deadzone').value
        rate           = self.get_parameter('control_rate_hz').value

        # ---- State ----
        self.current_joints = list(HOME_JOINTS)
        self.target_joints  = list(HOME_JOINTS)
        self.gripper_closed = False
        self.gripper_target = False   # True=close, False=open
        self.last_joints_time = None
        self.TIMEOUT_SEC = 1.0   # if no hand data, hold position

        # ---- Publishers (direct trajectory topic — works in Gazebo) ----
        self.arm_pub = self.create_publisher(
            JointTrajectory, ARM_TRAJ_TOPIC, 10)
        self.grip_pub = self.create_publisher(
            JointTrajectory, GRIPPER_TRAJ_TOPIC, 10)
        self.status_pub = self.create_publisher(String, '/hand_control_status', 10)

        # ---- Action clients (optional, for real robot) ----
        if self.use_ac:
            self._arm_ac = ActionClient(self, FollowJointTrajectory, ARM_ACTION)
            self._grip_ac = ActionClient(self, FollowJointTrajectory, GRIPPER_ACTION)

        # ---- Subscribers ----
        self.create_subscription(
            Float32MultiArray, '/arm_joints', self._cb_joints, 10)
        self.create_subscription(
            Float32, '/gripper_command', self._cb_gripper, 10)

        # ---- Control loop timer ----
        self.timer = self.create_timer(1.0 / rate, self._control_loop)

        # ---- Move to home on start ----
        self._send_arm_trajectory(HOME_JOINTS, duration=2.0)
        self._send_gripper_trajectory(GRIPPER_OPEN, duration=1.0)

        self.get_logger().info('✅ ur5_hand_controller_node started')
        self.get_logger().info(f'   Arm topic:     {ARM_TRAJ_TOPIC}')
        self.get_logger().info(f'   Gripper topic: {GRIPPER_TRAJ_TOPIC}')

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------
    def _cb_joints(self, msg: Float32MultiArray):
        if len(msg.data) < 6:
            return
        self.target_joints = list(msg.data[:6])
        self.last_joints_time = time.time()

    def _cb_gripper(self, msg: Float32):
        score = msg.data
        # Hysteresis: avoid chattering
        if score > self.g_close:
            self.gripper_target = True
        elif score < self.g_open:
            self.gripper_target = False
        # Between thresholds → keep current state (hysteresis)

    # -----------------------------------------------------------------------
    # Control loop
    # -----------------------------------------------------------------------
    def _control_loop(self):
        now = time.time()

        # ---- Timeout: if hand lost, hold ----
        if self.last_joints_time is None or (now - self.last_joints_time) > self.TIMEOUT_SEC:
            self._publish_status('WAITING - no hand detected')
            return

        # ---- Rate-limit joint deltas (smooth motion) ----
        new_joints = []
        changed = False
        for i, (curr, tgt) in enumerate(zip(self.current_joints, self.target_joints)):
            lo, hi = JOINT_LIMITS[i]
            tgt_clamped = float(np.clip(tgt, lo, hi))
            delta = tgt_clamped - curr
            if abs(delta) < self.deadzone:
                new_joints.append(curr)
            else:
                step = np.clip(delta, -self.max_delta, self.max_delta)
                new_joints.append(curr + step)
                changed = True

        self.current_joints = new_joints

        # ---- Send arm trajectory ----
        if changed:
            self._send_arm_trajectory(self.current_joints, duration=self.traj_dur)

        # ---- Gripper ----
        if self.gripper_target != self.gripper_closed:
            pos = GRIPPER_CLOSED if self.gripper_target else GRIPPER_OPEN
            self._send_gripper_trajectory(pos, duration=0.5)
            self.gripper_closed = self.gripper_target
            state = 'CLOSE' if self.gripper_target else 'OPEN'
            self.get_logger().info(f'🤖 Gripper → {state}')

        g_str = 'CLOSED' if self.gripper_closed else 'OPEN'
        deg = [f'{math.degrees(j):.1f}°' for j in self.current_joints]
        self._publish_status(f'ACTIVE | Gripper:{g_str} | J={deg}')

    # -----------------------------------------------------------------------
    # Trajectory helpers
    # -----------------------------------------------------------------------
    def _send_arm_trajectory(self, joints, duration=0.5):
        """Publish a single-point JointTrajectory to the arm controller."""
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names  = UR5_JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions  = [float(j) for j in joints]
        pt.velocities = [0.0] * 6
        pt.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9))

        traj.points = [pt]
        self.arm_pub.publish(traj)

    def _send_gripper_trajectory(self, position, duration=0.5):
        """Publish a single-point JointTrajectory to the gripper controller."""
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names  = [GRIPPER_JOINT]

        pt = JointTrajectoryPoint()
        pt.positions  = [float(position)]
        pt.velocities = [0.0]
        pt.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9))

        traj.points = [pt]
        self.grip_pub.publish(traj)

    # -----------------------------------------------------------------------
    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = UR5HandControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()