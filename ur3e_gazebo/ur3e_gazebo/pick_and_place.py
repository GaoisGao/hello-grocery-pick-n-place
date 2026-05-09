"""
pick_and_place.py
=================
Sequences the UR3e arm through a full pick-and-place motion:

  1. Open gripper        (at home pose)
  2. Move to approach    (above the apple, gripper open)
  3. Move to grasp       (gripper fingers around the apple, still open)
  4. Close gripper       (grip the apple)
  5. Lift                (raise apple off the table)
  6. Transport           (rotate arm to face the shelf)
  7. Place               (lower apple onto the shelf board)
  8. Open gripper        (release)
  9. Return to home

USAGE
-----
  Launch the simulation first, click Play, wait for the arm to reach home
  pose, then run:

    ros2 run ur3e_gazebo pick_and_place

TUNING JOINT ANGLES — PHASE 8.3
---------------------------------
The joint angles below are initial estimates. They need to be verified and
tuned in the simulation before the sequence will work correctly.

For each pose:
  1. Use `ros2 topic pub --once` to send the arm to approximately the right
     position (example commands are in notes/ros-gazebo-v1/Gripper Testing Guide.md)
  2. Fine-tune by eye in the Gazebo GUI
  3. Record the actual joint angles:
       ros2 topic echo /joint_states --once
  4. Copy the `position` values into the matching constant below.

NOTE: /joint_states reports joints in alphabetical order:
  elbow_joint, left_finger_joint, right_finger_joint,
  shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint,
  wrist_2_joint, wrist_3_joint
Match each value to the correct joint name when copying.

WORLD GEOMETRY (for reference while tuning)
-------------------------------------------
  Apple center:       world (0.35, 0.0, 0.065)   — on the table surface
  Table surface:      world z ≈ 0.025            — top face of the table box
  Shelf bottom board: world (≈-0.40, 0.0, 0.015) — bottom shelf near floor
  Shelf middle board: world (≈-0.40, 0.0, 0.300) — middle shelf at 30 cm
  Arm base:           world (0.0, 0.0, 0.0)       — robot mount point
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# ═══════════════════════════════════════════════════════════════════════════════
#  TIMING  — edit these to adjust the speed of the sequence
# ═══════════════════════════════════════════════════════════════════════════════

MOVE_DURATION_SEC = 4.0   # seconds given to the arm to reach each pose
HOLD_SEC          = 2.0   # seconds to wait after arriving before the next step


# ═══════════════════════════════════════════════════════════════════════════════
#  JOINT ANGLE CONSTANTS  — recorded in Phase 8.3 via joint_control_panel
#
#  Each dict maps joint name → position in radians (arm) or meters (fingers).
# ═══════════════════════════════════════════════════════════════════════════════

# Step 1 / Step 10 — home pose, facing the table, gripper neutral.
_HOME = {
    'shoulder_pan_joint':   0.000,
    'shoulder_lift_joint': -1.570,
    'elbow_joint':          1.570,
    'wrist_1_joint':       -1.570,
    'wrist_2_joint':        0.000,
    'wrist_3_joint':        0.000,
}

# Step 2 — arm swung to a position just short of being above the apple,
# gripper already open so it clears the apple on the way down.
_APPROACH = {
    'shoulder_pan_joint':  -0.385,
    'shoulder_lift_joint': -1.232,
    'elbow_joint':          1.352,
    'wrist_1_joint':       -1.570,
    'wrist_2_joint':       -1.571,
    'wrist_3_joint':        0.000,
}

# Step 3 — arm lowered so the open gripper is positioned around the apple.
_GRASP = {
    'shoulder_pan_joint':  -0.385,
    'shoulder_lift_joint': -1.232,
    'elbow_joint':          1.706,
    'wrist_1_joint':       -1.953,
    'wrist_2_joint':       -1.571,
    'wrist_3_joint':        0.000,
}

# Step 5 — arm raised with apple gripped, high enough to clear the table
# and rotate to the shelf without collision.
_LIFT = {
    'shoulder_pan_joint':  -0.385,
    'shoulder_lift_joint': -2.226,
    'elbow_joint':          1.425,
    'wrist_1_joint':       -1.953,
    'wrist_2_joint':       -1.571,
    'wrist_3_joint':        0.000,
}

# Step 6 — arm rotated to face the shelf, still held high.
_TRANSPORT = {
    'shoulder_pan_joint':  -3.283,
    'shoulder_lift_joint': -2.120,
    'elbow_joint':          1.425,
    'wrist_1_joint':       -1.953,
    'wrist_2_joint':       -1.571,
    'wrist_3_joint':        0.000,
}

# Step 7 — arm lowered to deposit the apple on the shelf board.
_PLACE = {
    'shoulder_pan_joint':  -3.283,
    'shoulder_lift_joint': -1.754,
    'elbow_joint':          1.425,
    'wrist_1_joint':       -1.953,
    'wrist_2_joint':       -1.571,
    'wrist_3_joint':        0.000,
}

# Step 9 — arm raised clear of the shelf before swinging back to home.
_RETRACT = {
    'shoulder_pan_joint':  -3.283,
    'shoulder_lift_joint': -2.371,
    'elbow_joint':          1.425,
    'wrist_1_joint':       -1.953,
    'wrist_2_joint':       -1.571,
    'wrist_3_joint':        0.000,
}


# ═══════════════════════════════════════════════════════════════════════════════
#  SEQUENCE DEFINITION
#
#  Each step is a tuple:
#    (step_name, arm_joints_dict, finger_position, move_duration_sec, hold_sec)
#
#  finger_position (meters): 0.0 = neutral, 0.05 = fully open, -0.006 = gripping
#  move_duration:  time allowed for the arm to reach the target pose
#  hold_sec:       additional wait after the move completes before next step
# ═══════════════════════════════════════════════════════════════════════════════

_OPEN  =  0.05    # gripper fully open
_GRIP  = -0.006   # gripper squeezing apple (tuned value)
_NEUTRAL = 0.0    # gripper at rest / home

_M = MOVE_DURATION_SEC
_H = HOLD_SEC

SEQUENCE = [
    # name                  arm joints    fingers    move_dur  hold
    ('open_gripper',       _HOME,         _OPEN,     _M,       _H),
    ('approach',           _APPROACH,     _OPEN,     _M,       _H),
    ('grasp_position',     _GRASP,        _OPEN,     _M,       _H),
    ('close_gripper',      _GRASP,        _GRIP,     _M,       _H),
    ('lift',               _LIFT,         _GRIP,     _M,       _H),
    ('transport',          _TRANSPORT,    _GRIP,     _M,       _H),
    ('place',              _PLACE,        _GRIP,     _M,       _H),
    ('open_gripper',       _PLACE,        _OPEN,     _M,       _H),
    ('retract',            _RETRACT,      _OPEN,     _M,       _H),
    ('home',               _HOME,         _NEUTRAL,  _M,       _H),
]


# ═══════════════════════════════════════════════════════════════════════════════
#  NODE
# ═══════════════════════════════════════════════════════════════════════════════

class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__('pick_and_place')

        self._pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10,
        )

        self._step = 0
        self._step_start = None   # wall-clock time when current step started

        # Tick at 10 Hz — checks whether the current step is done and advances
        self._timer = self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'pick_and_place started — {len(SEQUENCE)} steps in sequence.'
        )

    # ------------------------------------------------------------------ #

    def _tick(self):
        import time

        now = time.monotonic()

        if self._step >= len(SEQUENCE):
            self.get_logger().info('Sequence complete.')
            raise SystemExit

        name, arm_joints, finger_pos, move_dur, hold_sec = SEQUENCE[self._step]
        total_step_sec = move_dur + hold_sec

        if self._step_start is None:
            # First tick for this step — send the trajectory command
            self._send(arm_joints, finger_pos, move_dur)
            self._step_start = now
            self.get_logger().info(
                f'[{self._step + 1}/{len(SEQUENCE)}] {name} '
                f'(move {move_dur}s + hold {hold_sec}s)'
            )
        elif now - self._step_start >= total_step_sec:
            # Step complete — advance
            self._step += 1
            self._step_start = None

    # ------------------------------------------------------------------ #

    def _send(self, arm_joints: dict, finger_pos: float, duration_sec: float):
        """
        Publishes a single JointTrajectory command combining the arm pose
        and the desired finger position.
        """
        joint_names = list(arm_joints.keys()) + ['left_finger_joint', 'right_finger_joint']
        positions   = list(arm_joints.values()) + [finger_pos, finger_pos]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec),
                                         nanosec=int((duration_sec % 1) * 1e9))

        msg = JointTrajectory()
        msg.joint_names = joint_names
        msg.points = [point]

        self._pub.publish(msg)


# ═══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
