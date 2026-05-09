#!/usr/bin/env python3

from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    PlanningOptions,
)


class PickState(Enum):
    IDLE = auto()
    MOVE_TO_PRE_GRASP = auto()
    MOVE_TO_GRASP = auto()
    CLOSE_GRIPPER = auto()
    MOVE_TO_LIFT = auto()
    DONE = auto()
    ABORTED = auto()


class PickExecutor(Node):
    """
    Executes the pick sequence as a small state machine.

    Required input topics:
      /pre_grasp_pose : PoseStamped
      /grasp_pose     : PoseStamped
      /lift_pose      : PoseStamped
      /start_pick     : Bool

    MoveIt:
      Sends pose goals to /move_action.

    Gripper:
      Publishes Bool to /gripper/close.
      You must adapt this topic/message to your real gripper controller.
    """

    def __init__(self):
        super().__init__("pick_executor")

        # ---- MoveIt parameters ----
        self.declare_parameter("move_group_action", "/move_action")
        self.declare_parameter("planning_group", "ur_manipulator")
        self.declare_parameter("eef_link", "tool0")

        # For first testing, keep plan_only=True.
        # Set to False only after RViz checks look correct.
        self.declare_parameter("plan_only", True)

        # Conservative speed for real hardware.
        self.declare_parameter("velocity_scaling", 0.05)
        self.declare_parameter("acceleration_scaling", 0.05)
        self.declare_parameter("allowed_planning_time", 5.0)

        # ---- Constraint tolerances ----
        # Position tolerance is radius of sphere around target pose.
        # If too small, MoveIt may fail. If too large, robot may not go close enough.
        self.declare_parameter("position_tolerance", 0.02)

        # Orientation tolerance in radians.
        # Increase if IK fails due to strict orientation requirement.
        self.declare_parameter("orientation_tolerance", 0.35)

        # ---- Gripper parameters ----
        # Placeholder gripper interface.
        # Replace /gripper/close with your actual gripper topic/action/service.
        self.declare_parameter("gripper_close_topic", "/gripper/close")

        # Time to wait after commanding gripper close.
        # Tune based on servo/gripper speed.
        self.declare_parameter("gripper_close_wait_s", 1.0)

        self.move_group_action = self.get_parameter("move_group_action").value
        self.planning_group = self.get_parameter("planning_group").value
        self.eef_link = self.get_parameter("eef_link").value
        self.plan_only = bool(self.get_parameter("plan_only").value)

        self.velocity_scaling = float(self.get_parameter("velocity_scaling").value)
        self.acceleration_scaling = float(self.get_parameter("acceleration_scaling").value)
        self.allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)

        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.orientation_tolerance = float(self.get_parameter("orientation_tolerance").value)

        self.gripper_close_topic = self.get_parameter("gripper_close_topic").value
        self.gripper_close_wait_s = float(self.get_parameter("gripper_close_wait_s").value)

        self.state = PickState.IDLE
        self.busy = False

        self.pre_grasp_pose: Optional[PoseStamped] = None
        self.grasp_pose: Optional[PoseStamped] = None
        self.lift_pose: Optional[PoseStamped] = None

        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            self.move_group_action,
        )

        self.create_subscription(PoseStamped, "/pre_grasp_pose", self.pre_grasp_callback, 10)
        self.create_subscription(PoseStamped, "/grasp_pose", self.grasp_callback, 10)
        self.create_subscription(PoseStamped, "/lift_pose", self.lift_callback, 10)
        self.create_subscription(Bool, "/start_pick", self.start_pick_callback, 10)

        self.gripper_pub = self.create_publisher(Bool, self.gripper_close_topic, 10)

        self.get_logger().info("Pick executor started.")
        self.get_logger().info(f"MoveIt action: {self.move_group_action}")
        self.get_logger().info(f"Planning group: {self.planning_group}")
        self.get_logger().info(f"EEF link: {self.eef_link}")
        self.get_logger().info(f"plan_only: {self.plan_only}")
        self.get_logger().info(f"Gripper close topic: {self.gripper_close_topic}")

    # -------------------------
    # Topic callbacks
    # -------------------------

    def pre_grasp_callback(self, msg: PoseStamped):
        self.pre_grasp_pose = msg

    def grasp_callback(self, msg: PoseStamped):
        self.grasp_pose = msg

    def lift_callback(self, msg: PoseStamped):
        self.lift_pose = msg

    def start_pick_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.busy:
            self.get_logger().warn("Pick sequence already running. Ignoring /start_pick.")
            return

        if not self.all_poses_ready():
            self.get_logger().error("Cannot start pick: missing one or more poses.")
            self.report_pose_status()
            return

        self.get_logger().warn("Starting pick sequence.")
        self.busy = True
        self.transition_to(PickState.MOVE_TO_PRE_GRASP)

    # -------------------------
    # State machine
    # -------------------------

    def transition_to(self, new_state: PickState):
        self.state = new_state
        self.get_logger().info(f"State → {self.state.name}")

        if self.state == PickState.MOVE_TO_PRE_GRASP:
            self.send_moveit_goal(self.pre_grasp_pose, next_state_on_success=PickState.MOVE_TO_GRASP)

        elif self.state == PickState.MOVE_TO_GRASP:
            self.send_moveit_goal(self.grasp_pose, next_state_on_success=PickState.CLOSE_GRIPPER)

        elif self.state == PickState.CLOSE_GRIPPER:
            self.close_gripper()

        elif self.state == PickState.MOVE_TO_LIFT:
            self.send_moveit_goal(self.lift_pose, next_state_on_success=PickState.DONE)

        elif self.state == PickState.DONE:
            self.get_logger().info("Pick sequence completed successfully.")
            self.busy = False

        elif self.state == PickState.ABORTED:
            self.get_logger().error("Pick sequence aborted.")
            self.busy = False

    def close_gripper(self):
        if self.plan_only:
            self.get_logger().warn("plan_only=True, skipping gripper close.")
            self.next_state_timer = self.create_timer(
                1.0,
                lambda: self.delayed_transition_once(PickState.MOVE_TO_LIFT)
            )
            return

        self.get_logger().warn("Closing gripper.")

        cmd = Bool()
        cmd.data = True
        self.gripper_pub.publish(cmd)

        # Simple wait timer before moving to lift.
        # Replace this with actual gripper feedback later.
        self.create_timer(self.gripper_close_wait_s, self._finish_gripper_close_once)

    def _finish_gripper_close_once(self):
        # This timer callback may repeat, so guard it.
        if self.state != PickState.CLOSE_GRIPPER:
            return

        self.get_logger().info("Assuming gripper close completed.")
        self.transition_to(PickState.MOVE_TO_LIFT)

    # -------------------------
    # MoveIt action
    # -------------------------

    def send_moveit_goal(self, pose: PoseStamped, next_state_on_success: PickState):
        if pose is None:
            self.get_logger().error("Pose is None. Aborting.")
            self.transition_to(PickState.ABORTED)
            return

        self.current_next_state = next_state_on_success

        self.get_logger().info(
            f"Sending MoveIt goal for state {self.state.name}: "
            f"frame={pose.header.frame_id}, "
            f"x={pose.pose.position.x:.3f}, "
            f"y={pose.pose.position.y:.3f}, "
            f"z={pose.pose.position.z:.3f}"
        )

        if pose.header.frame_id == "":
            self.get_logger().error("Pose frame_id is empty. Aborting.")
            self.transition_to(PickState.ABORTED)
            return

        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"MoveGroup action server not available: {self.move_group_action}")
            self.transition_to(PickState.ABORTED)
            return

        goal_msg = self.make_move_group_goal(pose)

        future = self.move_group_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def make_move_group_goal(self, target_pose: PoseStamped) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()

        req = goal.request
        req.group_name = self.planning_group
        req.num_planning_attempts = 10
        req.allowed_planning_time = self.allowed_planning_time
        req.max_velocity_scaling_factor = self.velocity_scaling
        req.max_acceleration_scaling_factor = self.acceleration_scaling

        constraints = Constraints()
        constraints.name = f"{self.state.name}_constraint"

        # Position constraint
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.position_tolerance]

        sphere_pose = Pose()
        sphere_pose.position = target_pose.pose.position
        sphere_pose.orientation.w = 1.0

        region = BoundingVolume()
        region.primitives.append(sphere)
        region.primitive_poses.append(sphere_pose)

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = target_pose.header.frame_id
        position_constraint.link_name = self.eef_link
        position_constraint.constraint_region = region
        position_constraint.weight = 1.0

        constraints.position_constraints.append(position_constraint)

        # Orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = target_pose.header.frame_id
        orientation_constraint.link_name = self.eef_link
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = self.orientation_tolerance
        orientation_constraint.weight = 1.0

        constraints.orientation_constraints.append(orientation_constraint)

        req.goal_constraints.append(constraints)

        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = self.plan_only
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        return goal

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"MoveIt rejected goal in state {self.state.name}.")
            self.transition_to(PickState.ABORTED)
            return

        self.get_logger().info(f"MoveIt accepted goal in state {self.state.name}.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        try:
            result = future.result().result
            error_code = result.error_code.val
        except Exception as e:
            self.get_logger().error(f"Failed to get MoveIt result: {e}")
            self.transition_to(PickState.ABORTED)
            return

        if error_code == 1:
            self.get_logger().info(f"MoveIt success in state {self.state.name}.")

            next_state = self.current_next_state

            # Do not immediately send the next MoveIt goal inside the result callback.
            # Give the action client / MoveIt / DDS layer time to finish cleanup.
            self.next_state_timer = self.create_timer(
                1.0,
                lambda: self.delayed_transition_once(next_state)
            )
        else:
            self.get_logger().error(
                f"MoveIt failed in state {self.state.name} with error_code={error_code}"
            )
            self.transition_to(PickState.ABORTED)

    # -------------------------
    # Helpers
    # -------------------------

    def all_poses_ready(self) -> bool:
        return (
            self.pre_grasp_pose is not None
            and self.grasp_pose is not None
            and self.lift_pose is not None
        )

    def report_pose_status(self):
        self.get_logger().info(f"pre_grasp_pose ready: {self.pre_grasp_pose is not None}")
        self.get_logger().info(f"grasp_pose ready: {self.grasp_pose is not None}")
        self.get_logger().info(f"lift_pose ready: {self.lift_pose is not None}")

    def delayed_transition_once(self, next_state):
        if hasattr(self, "next_state_timer"):
            self.next_state_timer.cancel()
            self.destroy_timer(self.next_state_timer)

        self.transition_to(next_state)

def main(args=None):
    rclpy.init(args=args)
    node = PickExecutor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()