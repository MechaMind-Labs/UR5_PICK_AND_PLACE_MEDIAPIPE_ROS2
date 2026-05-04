#!/usr/bin/env python3
"""
hand_detection_node.py
======================
Node 1 - Hand & Pose Detection Publisher

Uses MediaPipe Holistic (or Pose + Hands) to detect:
  - Full body pose landmarks (shoulder, elbow, wrist) → /hand_pose
  - Hand landmarks for finger tip detection  → /gripper_command

Published Topics:
  /hand_pose        → geometry_msgs/PoseArray  (shoulder[0], elbow[1], wrist[2])
  /arm_joints       → std_msgs/Float32MultiArray (6 normalised joint angles)
  /gripper_command  → std_msgs/Float32 (0.0 = open, 1.0 = close)
  /hand_image       → sensor_msgs/Image (annotated debug feed)

Author: MechaMind-Labs extension
"""

import rclpy
from rclpy.node import Node

import cv2 #4.7.0 version
import mediapipe as mp 
import numpy as np #1.26.4 version
import math

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# ---------------------------------------------------------------------------
# MediaPipe setup
# ---------------------------------------------------------------------------
mp_pose    = mp.solutions.pose
mp_hands   = mp.solutions.hands
mp_draw    = mp.solutions.drawing_utils
mp_styles  = mp.solutions.drawing_styles


# ---------------------------------------------------------------------------
# Helper geometry
# ---------------------------------------------------------------------------
def vec3(lm):
    """Return (x, y, z) from a landmark."""
    return np.array([lm.x, lm.y, lm.z])


def angle_between(a, b, c):
    """Angle at joint b given three 3-D points a-b-c (degrees)."""
    ba = a - b
    bc = c - b
    cos_ang = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-8)
    return math.degrees(math.acos(np.clip(cos_ang, -1.0, 1.0)))


def fingertip_convergence(hand_lms):
    """
    Returns 0.0 (open) to 1.0 (fully closed / pinch) based on mean
    distance of all 4 finger-tips to the thumb tip.

    Landmark indices:
      4=THUMB_TIP, 8=INDEX_TIP, 12=MIDDLE_TIP, 16=RING_TIP, 20=PINKY_TIP
    Wrist = 0, used for normalisation.
    """
    ids  = [4, 8, 12, 16, 20]
    tips = [vec3(hand_lms.landmark[i]) for i in ids]
    wrist = vec3(hand_lms.landmark[0])

    # normalise by wrist-to-middle-finger-mcp distance (landmark 9)
    ref_dist = np.linalg.norm(vec3(hand_lms.landmark[9]) - wrist) + 1e-6

    # mean distance between thumb tip and every other finger tip
    thumb = tips[0]
    dists = [np.linalg.norm(thumb - tips[i]) / ref_dist for i in range(1, 5)]
    mean_dist = np.mean(dists)

    # heuristic: when mean_dist < 0.4 → pinch/close, > 1.2 → open
    score = 1.0 - np.clip((mean_dist - 0.4) / 0.8, 0.0, 1.0)
    return float(score)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class HandDetectionNode(Node):

    def __init__(self):
        super().__init__('hand_detection_node')

        # ---- Parameters ----
        self.declare_parameter('camera_index',   0)
        self.declare_parameter('mirror',         True)   # mirror for intuitive control
        self.declare_parameter('use_right_hand', True)   # right-hand pose landmarks
        self.declare_parameter('smoothing',      0.4)    # EMA alpha (0=no smooth, 1=frozen)
        self.declare_parameter('show_window',    True)

        cam_idx      = self.get_parameter('camera_index').value
        self.mirror  = self.get_parameter('mirror').value
        self.rhand   = self.get_parameter('use_right_hand').value
        alpha        = self.get_parameter('smoothing').value
        self.show_win = self.get_parameter('show_window').value

        self.alpha   = alpha          # EMA smoothing factor
        self.prev_joints = None
        self.prev_grip   = 0.0

        # ---- Publishers ----
        self.pub_pose    = self.create_publisher(PoseArray,         '/hand_pose',        10)
        self.pub_joints  = self.create_publisher(Float32MultiArray, '/arm_joints',       10)
        self.pub_gripper = self.create_publisher(Float32,           '/gripper_command',  10)
        self.pub_image   = self.create_publisher(Image,             '/hand_image',       10)

        self.bridge = CvBridge()

        # ---- Camera ----
        self.cap = cv2.VideoCapture(cam_idx)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera {cam_idx}')
            raise RuntimeError('Camera open failed')

        # ---- MediaPipe ----
        self.pose  = mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.5)

        self.hands = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.5)

        # ---- Timer: ~30 Hz ----
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)
        self.get_logger().info('✅ hand_detection_node started')

    # -----------------------------------------------------------------------
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Empty frame')
            return

        if self.mirror:
            frame = cv2.flip(frame, 1)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False

        pose_result  = self.pose.process(rgb)
        hands_result = self.hands.process(rgb)

        rgb.flags.writeable = True
        frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # ---- Draw pose ----
        if pose_result.pose_landmarks:
            mp_draw.draw_landmarks(
                frame, pose_result.pose_landmarks,
                mp_pose.POSE_CONNECTIONS,
                mp_draw.DrawingSpec(color=(0, 255, 120), thickness=2, circle_radius=3),
                mp_draw.DrawingSpec(color=(255, 80, 0),  thickness=2))

        # ---- Draw hand ----
        if hands_result.multi_hand_landmarks:
            for hl in hands_result.multi_hand_landmarks:
                mp_draw.draw_landmarks(
                    frame, hl,
                    mp_hands.HAND_CONNECTIONS,
                    mp_styles.get_default_hand_landmarks_style(),
                    mp_styles.get_default_hand_connections_style())

        # ---- Compute & publish ----
        joints  = self._extract_arm_joints(pose_result)
        gripper = self._extract_gripper(hands_result)

        if joints is not None:
            self._publish_joints(joints)
            self._publish_pose(pose_result)
            self._draw_hud(frame, joints, gripper)

        self._publish_gripper(gripper)

        # ---- Image topic ----
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_image.publish(img_msg)

        # ---- Optional window ----
        if self.show_win:
            cv2.imshow('Hand Control - UR5', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

    # -----------------------------------------------------------------------
    # Joint extraction
    # -----------------------------------------------------------------------
    def _extract_arm_joints(self, pose_result):
        """
        Maps human arm pose (shoulder→elbow→wrist) to 6 UR5 joint angles.

        Joint mapping (intuitive):
          J1 (shoulder pan)   ← shoulder horizontal angle (left/right of body)
          J2 (shoulder lift)  ← shoulder vertical elevation (up/down)
          J3 (elbow)          ← elbow flexion angle
          J4 (wrist roll)     ← wrist roll estimated from forearm tilt
          J5 (wrist pitch)    ← wrist vertical deviation
          J6 (wrist yaw)      ← wrist yaw (hand lateral twist) — set 0 here
        """
        if not pose_result.pose_landmarks:
            return None

        lms = pose_result.pose_landmarks.landmark

        # Landmark indices
        if self.rhand:  # user's RIGHT arm
            sh = mp_pose.PoseLandmark.RIGHT_SHOULDER
            el = mp_pose.PoseLandmark.RIGHT_ELBOW
            wr = mp_pose.PoseLandmark.RIGHT_WRIST
            hp = mp_pose.PoseLandmark.RIGHT_HIP
        else:           # user's LEFT arm
            sh = mp_pose.PoseLandmark.LEFT_SHOULDER
            el = mp_pose.PoseLandmark.LEFT_ELBOW
            wr = mp_pose.PoseLandmark.LEFT_WRIST
            hp = mp_pose.PoseLandmark.LEFT_HIP

        p_sh = vec3(lms[sh])
        p_el = vec3(lms[el])
        p_wr = vec3(lms[wr])
        p_hp = vec3(lms[hp])

        # ---- J1: shoulder pan (left-right) ----
        # Horizontal angle of upper-arm relative to body midline
        shoulder_vec   = p_el - p_sh
        j1 = math.atan2(shoulder_vec[0], -shoulder_vec[2])  # horizontal plane
        j1 = np.clip(j1, -math.pi, math.pi)

        # ---- J2: shoulder lift (up-down elevation) ----
        # Angle from horizontal: negative = arm up (UR5 convention)
        j2 = math.atan2(-shoulder_vec[1], math.sqrt(shoulder_vec[0]**2 + shoulder_vec[2]**2))
        j2 = np.clip(j2, -math.pi, math.pi)

        # ---- J3: elbow flexion ----
        elbow_ang_deg = angle_between(p_sh, p_el, p_wr)   # 0=straight, 180=folded
        # UR5 J3 range roughly -π to 0; map 180deg→0, 90deg→-π/2
        j3 = -math.radians(180 - elbow_ang_deg)
        j3 = np.clip(j3, -math.pi, 0.0)

        # ---- J4: wrist roll (forearm axial rotation) ----
        # Estimate from wrist-to-elbow vector vertical component
        forearm = p_wr - p_el
        j4 = math.atan2(forearm[1], forearm[0])
        j4 = np.clip(j4, -math.pi, math.pi)

        # ---- J5: wrist pitch ----
        # Elevation of forearm relative to elbow
        j5 = math.atan2(-forearm[1], math.sqrt(forearm[0]**2 + forearm[2]**2))
        j5 = np.clip(j5, -math.pi/2, math.pi/2)

        # ---- J6: wrist yaw (static or from hand landmarks) ----
        j6 = 0.0

        raw = [j1, j2, j3, j4, j5, j6]

        # ---- EMA smoothing ----
        if self.prev_joints is None:
            self.prev_joints = raw
        smooth = [self.alpha * self.prev_joints[i] + (1 - self.alpha) * raw[i]
                  for i in range(6)]
        self.prev_joints = smooth

        return smooth

    # -----------------------------------------------------------------------
    def _extract_gripper(self, hands_result):
        """Returns 0.0=open, 1.0=closed based on finger convergence."""
        if not hands_result.multi_hand_landmarks:
            raw = 0.0   # no hand visible → open
        else:
            raw = fingertip_convergence(hands_result.multi_hand_landmarks[0])

        # EMA
        smooth = self.alpha * self.prev_grip + (1 - self.alpha) * raw
        self.prev_grip = smooth
        return smooth

    # -----------------------------------------------------------------------
    # Publishers
    # -----------------------------------------------------------------------
    def _publish_joints(self, joints):
        msg = Float32MultiArray()
        msg.data = [float(j) for j in joints]
        self.pub_joints.publish(msg)

    def _publish_pose(self, pose_result):
        if not pose_result.pose_landmarks:
            return
        lms = pose_result.pose_landmarks.landmark

        if self.rhand:
            idxs = [mp_pose.PoseLandmark.RIGHT_SHOULDER,
                    mp_pose.PoseLandmark.RIGHT_ELBOW,
                    mp_pose.PoseLandmark.RIGHT_WRIST]
        else:
            idxs = [mp_pose.PoseLandmark.LEFT_SHOULDER,
                    mp_pose.PoseLandmark.LEFT_ELBOW,
                    mp_pose.PoseLandmark.LEFT_WRIST]

        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'camera_frame'
        for idx in idxs:
            p = Pose()
            lm = lms[idx]
            p.position.x = lm.x
            p.position.y = lm.y
            p.position.z = lm.z
            pa.poses.append(p)
        self.pub_pose.publish(pa)

    def _publish_gripper(self, score):
        msg = Float32()
        msg.data = float(score)
        self.pub_gripper.publish(msg)

    # -----------------------------------------------------------------------
    def _draw_hud(self, frame, joints, gripper):
        h, w = frame.shape[:2]
        deg = [math.degrees(j) for j in joints]
        labels = ['J1 Pan', 'J2 Lift', 'J3 Elbow', 'J4 WRoll', 'J5 WPitch', 'J6 WYaw']
        cv2.rectangle(frame, (0, 0), (260, 200), (0, 0, 0), -1)
        for i, (l, d) in enumerate(zip(labels, deg)):
            cv2.putText(frame, f'{l}: {d:+.1f}°',
                        (8, 22 + i * 26), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 255, 200), 1, cv2.LINE_AA)

        # Gripper bar
        bar_w = int(gripper * 200)
        color = (0, 80, 255) if gripper > 0.6 else (0, 220, 0)
        cv2.rectangle(frame, (8, 178), (208, 196), (80, 80, 80), -1)
        cv2.rectangle(frame, (8, 178), (8 + bar_w, 196), color, -1)
        state = 'CLOSE' if gripper > 0.6 else 'OPEN'
        cv2.putText(frame, f'Gripper: {state} ({gripper:.2f})',
                    (8, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # -----------------------------------------------------------------------
    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = HandDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()