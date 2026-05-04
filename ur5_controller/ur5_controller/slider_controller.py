#!/usr/bin/env python3
"""
slider_controller.py
--------------------
Converts /joint_commands (from joint_state_publisher_gui) into
JointTrajectory messages for arm_controller and gripper_controller.

Jitter fixes applied here
──────────────────────────
1. time_from_start = 600 ms
     Gives Ignition physics enough time to actually reach the commanded
     position before the trajectory is considered "due".  With open_loop=true
     in the controller this value is mainly used as the trajectory window
     length — longer = smoother interpolation inside the JTC.

2. Rate-limited publish (max 20 Hz)
     joint_state_publisher_gui publishes at ~50 Hz when you drag a slider.
     Flooding the JTC with a new trajectory every 20 ms causes it to
     constantly restart its interpolation from scratch → jitter.
     We throttle to 20 Hz so the physics engine has 50 ms per command to
     make visible progress before the next command arrives.

3. Dead-band filter (1 mrad)
     Ignore slider noise smaller than 0.001 rad.  The GUI sometimes sends
     tiny position changes due to floating-point rounding even when the
     slider is not moving — these create micro-trajectories that look like
     vibration.
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

PUBLISH_RATE_HZ  = 20          # max commands/sec sent to the JTC
DEADBAND_RAD     = 0.001       # ignore position changes smaller than this
TIME_FROM_START_NS = 600_000_000  # 600 ms — trajectory window length


class SliderControl(Node):
    def __init__(self):
        super().__init__("slider_control")

        self.arm_pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 10)

        self.sub = self.create_subscription(
            JointState, "/joint_commands", self._cb, 10)

        # Rate limiter state
        self._min_dt = 1.0 / PUBLISH_RATE_HZ
        self._last_pub_time = 0.0

        # Last sent positions for dead-band comparison
        self._last_arm = list(ARM_HOME)
        self._last_grp = list(GRIPPER_HOME)

        self.get_logger().info(
            f"SliderControl ready — max {PUBLISH_RATE_HZ} Hz, "
            f"deadband {DEADBAND_RAD} rad, "
            f"time_from_start {TIME_FROM_START_NS/1e6:.0f} ms"
        )

    def _cb(self, msg: JointState):
        # ── Rate limit ───────────────────────────────────────────────────────
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self._last_pub_time) < self._min_dt:
            return
        self._last_pub_time = now

        pos = dict(zip(msg.name, msg.position))

        arm_positions = [pos.get(j, ARM_HOME[i])     for i, j in enumerate(ARM_JOINTS)]
        grp_positions = [pos.get(j, GRIPPER_HOME[i]) for i, j in enumerate(GRIPPER_JOINTS)]

        # ── Dead-band filter ─────────────────────────────────────────────────
        arm_changed = any(
            abs(a - b) > DEADBAND_RAD
            for a, b in zip(arm_positions, self._last_arm)
        )
        grp_changed = any(
            abs(a - b) > DEADBAND_RAD
            for a, b in zip(grp_positions, self._last_grp)
        )

        if arm_changed:
            self._publish(self.arm_pub, ARM_JOINTS, arm_positions)
            self._last_arm = list(arm_positions)

        if grp_changed:
            self._publish(self.gripper_pub, GRIPPER_JOINTS, grp_positions)
            self._last_grp = list(grp_positions)

    def _publish(self, pub, joint_names, positions):
        traj = JointTrajectory()
        traj.joint_names = joint_names
        traj.header.stamp = self.get_clock().now().to_msg()

        pt = JointTrajectoryPoint()
        pt.positions     = list(positions)
        pt.velocities    = [0.0] * len(positions)
        pt.accelerations = [0.0] * len(positions)
        pt.time_from_start = Duration(sec=0, nanosec=TIME_FROM_START_NS)

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
