#!/usr/bin/env python3
"""
home_pose.py

Sends a single JointTrajectory command to move the UR3e to its home pose
(arm facing the table at x=+0.6). This node fires once and then shuts down.

Called automatically by the launch file after joint_trajectory_controller
is active. You will see the arm move from its spawn position to this pose
once you click Play in Gazebo.

Home pose joint angles (radians):
  shoulder_pan_joint  :  0.0   (facing +X toward table)
  shoulder_lift_joint : -1.57  (arm sweeps forward)
  elbow_joint         :  1.57  (elbow bent, arm reaches out)
  wrist_1_joint       : -1.57  (wrist level)
  wrist_2_joint       :  0.0
  wrist_3_joint       :  0.0
  left_finger_joint   :  0.0   (gripper closed)
  right_finger_joint  :  0.0   (gripper closed)
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


HOME_POSE = {
    'shoulder_pan_joint':  0.0,
    'shoulder_lift_joint': -1.57,
    'elbow_joint':          1.57,
    'wrist_1_joint':       -1.57,
    'wrist_2_joint':        0.0,
    'wrist_3_joint':        0.0,
    'left_finger_joint':    0.0,
    'right_finger_joint':   0.0,
}

# Time allowed for the arm to reach home pose (seconds)
MOVE_DURATION_SEC = 3


class HomePoseNode(Node):
    def __init__(self):
        super().__init__('home_pose')

        self._pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10,
        )

        # Wait briefly for the controller to be ready, then send once
        self._timer = self.create_timer(1.0, self._send_home_pose)

    def _send_home_pose(self):
        # Cancel timer so this only fires once
        self._timer.cancel()

        joint_names = list(HOME_POSE.keys())
        positions   = list(HOME_POSE.values())

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=MOVE_DURATION_SEC, nanosec=0)

        msg = JointTrajectory()
        msg.joint_names = joint_names
        msg.points = [point]

        self._pub.publish(msg)
        self.get_logger().info(f'Home pose command sent — arm will reach pose in {MOVE_DURATION_SEC}s.')

        # Shut down after sending
        raise SystemExit


def main():
    rclpy.init()
    node = HomePoseNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
