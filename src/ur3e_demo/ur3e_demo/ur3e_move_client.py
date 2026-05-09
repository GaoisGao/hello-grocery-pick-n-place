#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    JointConstraint,
    Constraints,
    MoveItErrorCodes
)

# ================================
# 🔧 CONFIGURATION
# ================================

# Replace with your desired joint configs
HOME = [0.33,
    -2.11,
    -0.67,
    -1.36,
    0.67,
    0.00]
PREGRASP = [
    0.00,
    -1.57,
    0.00,
    -1.57,
    0.00,
    0.00
]
GRASP = [0.33,
    -2.11,
    -0.67,
    -1.36,
    0.67,
    0.00]

JOINT_NAMES = [
    "elbow_joint",
    "shoulder_lift_joint",
    "shoulder_pan_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


# ================================
# 🤖 ACTION CLIENT NODE
# ================================

class UR3eMoveClient(Node):

    def __init__(self):
        super().__init__("ur3e_move_client")

        self.client = ActionClient(self, MoveGroup, "/move_action")

        self.get_logger().info("Waiting for MoveIt action server...")
        self.client.wait_for_server()
        self.get_logger().info("Connected to MoveIt.\n")

    def move_to_joint_state(self, name, joint_positions):
        self.get_logger().info(f"Planning to: {name}")

        request = MotionPlanRequest()

        # Basic setup
        request.group_name = "ur_manipulator"
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3

        # Workspace bounds
        request.workspace_parameters.header.frame_id = "base_link"
        request.workspace_parameters.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
        request.workspace_parameters.max_corner = Vector3(x=1.0, y=1.0, z=1.0)

        # Joint constraints
        constraints = Constraints()

        for joint_name, pos in zip(JOINT_NAMES, joint_positions):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        request.goal_constraints.append(constraints)

        # Wrap into goal
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        goal.planning_options.planning_scene_diff.is_diff = True

        # Send goal
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected\n")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f"Reached {name}\n")
        else:
            self.get_logger().error(
                f"Failed {name}, error code: {result.error_code.val}\n"
            )


# ================================
# 🚀 MAIN EXECUTION
# ================================

def main():
    rclpy.init()

    node = UR3eMoveClient()

    # Simple sequence (you can change freely)
    node.move_to_joint_state("HOME", HOME)
    time.sleep(1)

    node.move_to_joint_state("PREGRASP", PREGRASP)
    time.sleep(1)

    node.move_to_joint_state("GRASP", GRASP)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
