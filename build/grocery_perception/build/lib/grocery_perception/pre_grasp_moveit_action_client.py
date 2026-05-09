#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    PlanningOptions,
)


class PreGraspMoveItActionClient(Node):
    def __init__(self):
        super().__init__("pre_grasp_moveit_action_client")

        self.declare_parameter("pose_topic", "/pre_grasp_pose")
        self.declare_parameter("move_group_action", "/move_action")
        self.declare_parameter("planning_group", "ur_manipulator")
        self.declare_parameter("eef_link", "tool0")
        self.declare_parameter("plan_only", True)
        self.declare_parameter("position_tolerance", 0.02)
        self.declare_parameter("orientation_tolerance", 0.30)
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("velocity_scaling", 0.05)
        self.declare_parameter("acceleration_scaling", 0.05)
        self.declare_parameter("run_once", True)

        self.pose_topic = self.get_parameter("pose_topic").value
        self.move_group_action = self.get_parameter("move_group_action").value
        self.planning_group = self.get_parameter("planning_group").value
        self.eef_link = self.get_parameter("eef_link").value
        self.plan_only = bool(self.get_parameter("plan_only").value)
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.orientation_tolerance = float(self.get_parameter("orientation_tolerance").value)
        self.allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
        self.velocity_scaling = float(self.get_parameter("velocity_scaling").value)
        self.acceleration_scaling = float(self.get_parameter("acceleration_scaling").value)
        self.run_once = bool(self.get_parameter("run_once").value)

        self.sent_goal = False
        self.busy = False

        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            self.move_group_action,
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10,
        )

        self.get_logger().info("Pre-grasp MoveIt action client started.")
        self.get_logger().info(f"Listening to pose topic: {self.pose_topic}")
        self.get_logger().info(f"MoveGroup action: {self.move_group_action}")
        self.get_logger().info(f"Planning group: {self.planning_group}")
        self.get_logger().info(f"End-effector link: {self.eef_link}")
        self.get_logger().info(f"Plan only: {self.plan_only}")

    def pose_callback(self, pose_msg: PoseStamped):
        if self.busy:
            return

        if self.run_once and self.sent_goal:
            return

        if pose_msg.header.frame_id == "":
            self.get_logger().error("Received pose with empty frame_id. Ignoring.")
            return

        self.busy = True
        self.sent_goal = True

        self.get_logger().info(
            f"Received pre-grasp pose in {pose_msg.header.frame_id}: "
            f"x={pose_msg.pose.position.x:.3f}, "
            f"y={pose_msg.pose.position.y:.3f}, "
            f"z={pose_msg.pose.position.z:.3f}"
        )

        goal_msg = self.make_move_group_goal(pose_msg)

        self.get_logger().info("Waiting for MoveGroup action server...")
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"MoveGroup action server not available at {self.move_group_action}"
            )
            self.busy = False
            return

        self.get_logger().info("Sending pre-grasp goal to MoveIt...")
        send_future = self.move_group_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def make_move_group_goal(self, target_pose: PoseStamped) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()

        req = goal.request
        req.group_name = self.planning_group
        req.num_planning_attempts = 5
        req.allowed_planning_time = self.allowed_planning_time
        req.max_velocity_scaling_factor = self.velocity_scaling
        req.max_acceleration_scaling_factor = self.acceleration_scaling

        # Position constraint: allow end-effector to be inside a small sphere
        # around the target pre-grasp position.
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.position_tolerance]

        region = BoundingVolume()
        region.primitives.append(sphere)
        region.primitive_poses.append(target_pose.pose)

        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = self.eef_link
        pos_constraint.constraint_region = region
        pos_constraint.weight = 1.0

        # Orientation constraint: use the orientation from /pre_grasp_pose.
        ori_constraint = OrientationConstraint()
        ori_constraint.header = target_pose.header
        ori_constraint.link_name = self.eef_link
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = self.orientation_tolerance
        ori_constraint.absolute_y_axis_tolerance = self.orientation_tolerance
        ori_constraint.absolute_z_axis_tolerance = self.orientation_tolerance
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.name = "pre_grasp_pose_goal"
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)

        req.goal_constraints.append(constraints)

        # Planning options
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = self.plan_only
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        # Important for MoveIt planning scene diff behavior
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        return goal

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("MoveIt rejected the pre-grasp goal.")
            self.busy = False
            return

        self.get_logger().info("MoveIt accepted the goal.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result

        error_code = result.error_code.val

        if error_code == 1:
            if self.plan_only:
                self.get_logger().info("MoveIt planning succeeded. No execution because plan_only=True.")
            else:
                self.get_logger().info("MoveIt planning/execution succeeded.")
        else:
            self.get_logger().error(f"MoveIt failed with error_code: {error_code}")

        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = PreGraspMoveItActionClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
