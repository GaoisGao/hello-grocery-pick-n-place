#!/usr/bin/env python3

import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    JointConstraint,
    Constraints,
    MoveItErrorCodes,
)


# ============================================================
# Joint order must match each position array
# ============================================================

JOINT_NAMES = [
    "elbow_joint",
    "shoulder_lift_joint",
    "shoulder_pan_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


PREGRASP = [
    -1.1986268758773804,
    -2.0385781727232875,
    0.1428239494562149,
    -1.4544995513609429,
    1.5791656970977783,
    0.013056532479822636,
]

GRASP = [
    -1.1275246143341064,
    -2.5302349529662074,
    0.12155867367982864,
    -0.9653489750674744,
    1.6738934516906738,
    0.05084680765867233,
]

PRE_PLACE = [
    -0.7715436816215515,
    -3.517334123651022,
    -2.240704361592428,
    0.9365362364002685,
    2.218136787414551,
    -0.17538148561586553,
]

PLACE = [
    -0.44914770126342773,
    -3.6867948971190394,
    -2.066961113606588,
    0.6458517748066406,
    1.9285905361175537,
    -0.1762483755694788,
]


# ============================================================
# Simple behavior-tree status
# ============================================================

class BTStatus(Enum):
    SUCCESS = auto()
    FAILURE = auto()


@dataclass
class BTNode:
    name: str

    def tick(self, ros_node: Node) -> BTStatus:
        raise NotImplementedError


@dataclass
class SequenceNode(BTNode):
    children: List[BTNode]

    def tick(self, ros_node: Node) -> BTStatus:
        ros_node.get_logger().warn(f"Starting sequence: {self.name}")

        for child in self.children:
            ros_node.get_logger().info(f"BT tick → {child.name}")
            status = child.tick(ros_node)

            if status != BTStatus.SUCCESS:
                ros_node.get_logger().error(f"BT node failed: {child.name}")
                return BTStatus.FAILURE

            ros_node.get_logger().info(f"BT node succeeded: {child.name}")
            time.sleep(0.5)

        ros_node.get_logger().warn(f"Sequence completed: {self.name}")
        return BTStatus.SUCCESS


@dataclass
class MoveJointNode(BTNode):
    joint_positions: List[float]

    def tick(self, ros_node: Node) -> BTStatus:
        success = ros_node.move_to_joint_state(self.name, self.joint_positions)
        return BTStatus.SUCCESS if success else BTStatus.FAILURE


@dataclass
class ManualGripperNode(BTNode):
    close: bool

    def tick(self, ros_node: Node) -> BTStatus:
        if self.close:
            input("\nPress ENTER to CLOSE the SO-101 gripper...")
            ros_node.publish_gripper_close()
        else:
            input("\nPress ENTER to OPEN the SO-101 gripper...")
            ros_node.publish_gripper_open()

        time.sleep(0.5)
        return BTStatus.SUCCESS


# ============================================================
# ROS 2 node
# ============================================================

class HardcodedPickBT(Node):
    def __init__(self):
        super().__init__("hardcoded_pick_bt")

        # -----------------------------
        # Tunable MoveIt parameters
        # -----------------------------
        self.declare_parameter("move_group_action", "/move_action")
        self.declare_parameter("planning_group", "ur_manipulator")
        self.declare_parameter("plan_only", False)

        # Tune lower for real hardware safety.
        self.declare_parameter("velocity_scaling", 0.10)
        self.declare_parameter("acceleration_scaling", 0.10)
        self.declare_parameter("allowed_planning_time", 5.0)

        # Joint tolerance for hard-coded target joint constraints.
        # If planning fails, try 0.02 or 0.03.
        self.declare_parameter("joint_tolerance", 0.01)

        # SO-101 gripper serial bridge topic.
        # This must match the subscriber topic of your SO-101 serial control node.
        self.declare_parameter("gripper_close_topic", "/gripper/close")

        self.move_group_action = self.get_parameter("move_group_action").value
        self.planning_group = self.get_parameter("planning_group").value
        self.plan_only = bool(self.get_parameter("plan_only").value)

        self.velocity_scaling = float(self.get_parameter("velocity_scaling").value)
        self.acceleration_scaling = float(self.get_parameter("acceleration_scaling").value)
        self.allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
        self.joint_tolerance = float(self.get_parameter("joint_tolerance").value)

        self.gripper_close_topic = self.get_parameter("gripper_close_topic").value

        self.moveit_client = ActionClient(self, MoveGroup, self.move_group_action)

        self.gripper_pub = self.create_publisher(
            Bool,
            self.gripper_close_topic,
            10,
        )

        self.get_logger().info("Hardcoded pick behavior tree started.")
        self.get_logger().info(f"MoveIt action: {self.move_group_action}")
        self.get_logger().info(f"Planning group: {self.planning_group}")
        self.get_logger().info(f"Plan only: {self.plan_only}")
        self.get_logger().info(f"Gripper topic: {self.gripper_close_topic}")

        self.get_logger().info("Waiting for MoveIt action server...")
        self.moveit_client.wait_for_server()
        self.get_logger().info("Connected to MoveIt.")

    # ------------------------------------------------------------
    # MoveIt joint target action
    # ------------------------------------------------------------

    def move_to_joint_state(self, target_name: str, joint_positions: List[float]) -> bool:
        if len(joint_positions) != len(JOINT_NAMES):
            self.get_logger().error(
                f"{target_name}: joint_positions length does not match JOINT_NAMES."
            )
            return False

        self.get_logger().warn(f"Planning to joint target: {target_name}")

        request = MotionPlanRequest()

        request.group_name = self.planning_group
        request.num_planning_attempts = 5
        request.allowed_planning_time = self.allowed_planning_time
        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = self.acceleration_scaling

        request.workspace_parameters.header.frame_id = "base_link"
        request.workspace_parameters.min_corner = Vector3(x=-1.5, y=-1.5, z=-1.5)
        request.workspace_parameters.max_corner = Vector3(x=1.5, y=1.5, z=1.5)

        constraints = Constraints()
        constraints.name = f"{target_name}_joint_constraints"

        for joint_name, joint_pos in zip(JOINT_NAMES, joint_positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = joint_pos
            joint_constraint.tolerance_above = self.joint_tolerance
            joint_constraint.tolerance_below = self.joint_tolerance
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        request.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = request

        goal.planning_options.plan_only = self.plan_only
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        goal.planning_options.look_around = False

        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        send_future = self.moveit_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(f"{target_name}: failed to send goal.")
            return False

        if not goal_handle.accepted:
            self.get_logger().error(f"{target_name}: MoveIt rejected goal.")
            return False

        self.get_logger().info(f"{target_name}: MoveIt accepted goal.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        error_code = result.error_code.val

        if error_code == MoveItErrorCodes.SUCCESS:
            if self.plan_only:
                self.get_logger().info(f"{target_name}: planning succeeded. No execution.")
            else:
                self.get_logger().info(f"{target_name}: reached target.")
            return True

        self.get_logger().error(
            f"{target_name}: MoveIt failed with error_code={error_code}"
        )
        return False

    # ------------------------------------------------------------
    # Gripper commands
    # ------------------------------------------------------------

    def publish_gripper_close(self):
        msg = Bool()
        msg.data = True
        self.gripper_pub.publish(msg)
        self.get_logger().warn("Published gripper CLOSE command.")

    def publish_gripper_open(self):
        msg = Bool()
        msg.data = False
        self.gripper_pub.publish(msg)
        self.get_logger().warn("Published gripper OPEN command.")

    # ------------------------------------------------------------
    # Behavior tree
    # ------------------------------------------------------------

    def build_tree(self) -> SequenceNode:
        return SequenceNode(
            name="Hardcoded Orange Pick and Place",
            children=[
                MoveJointNode("PREGRASP", PREGRASP),
                MoveJointNode("GRASP", GRASP),
                ManualGripperNode("CLOSE_GRIPPER", close=True),
                MoveJointNode("PRE_PLACE", PRE_PLACE),
                MoveJointNode("PLACE", PLACE),
                ManualGripperNode("OPEN_GRIPPER", close=False),
            ],
        )

    def run_tree(self):
        tree = self.build_tree()
        status = tree.tick(self)

        if status == BTStatus.SUCCESS:
            self.get_logger().warn("Pick-and-place behavior completed successfully.")
        else:
            self.get_logger().error("Pick-and-place behavior failed.")


def main(args=None):
    rclpy.init(args=args)

    node = HardcodedPickBT()

    try:
        input("\nPress ENTER to start the hardcoded pick-and-place behavior tree...")
        node.run_tree()
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted by user.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
