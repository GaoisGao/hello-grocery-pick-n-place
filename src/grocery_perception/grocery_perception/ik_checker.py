#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK


class IKChecker(Node):
    """
    Checks whether a PoseStamped target is IK-solvable by MoveIt.

    Subscribes:
      /pre_grasp_pose  or another PoseStamped topic

    Calls:
      /compute_ik

    Does not move the robot.
    """

    def __init__(self):
        super().__init__("ik_checker")

        self.declare_parameter("pose_topic", "/pre_grasp_pose")
        self.declare_parameter("planning_group", "ur_manipulator")
        self.declare_parameter("ik_link_name", "tool0")
        self.declare_parameter("ik_service", "/compute_ik")
        self.declare_parameter("timeout_sec", 1.0)
        self.declare_parameter("avoid_collisions", True)
        self.declare_parameter("run_once", False)

        self.pose_topic = self.get_parameter("pose_topic").value
        self.planning_group = self.get_parameter("planning_group").value
        self.ik_link_name = self.get_parameter("ik_link_name").value
        self.ik_service = self.get_parameter("ik_service").value
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.avoid_collisions = bool(self.get_parameter("avoid_collisions").value)
        self.run_once = bool(self.get_parameter("run_once").value)

        self.busy = False
        self.checked_once = False

        self.ik_client = self.create_client(GetPositionIK, self.ik_service)

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10,
        )

        self.get_logger().info("IK checker started.")
        self.get_logger().info(f"Listening to pose topic: {self.pose_topic}")
        self.get_logger().info(f"Planning group: {self.planning_group}")
        self.get_logger().info(f"IK link name: {self.ik_link_name}")
        self.get_logger().info(f"IK service: {self.ik_service}")
        self.get_logger().info(f"Avoid collisions: {self.avoid_collisions}")

    def pose_callback(self, pose_msg: PoseStamped):
        if self.busy:
            return

        if self.run_once and self.checked_once:
            return

        if pose_msg.header.frame_id == "":
            self.get_logger().error("Received pose with empty frame_id. Ignoring.")
            return

        if not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f"IK service not available: {self.ik_service}")
            return

        self.busy = True
        self.checked_once = True

        request = GetPositionIK.Request()

        request.ik_request.group_name = self.planning_group
        request.ik_request.ik_link_name = self.ik_link_name
        request.ik_request.pose_stamped = pose_msg
        request.ik_request.avoid_collisions = self.avoid_collisions

        request.ik_request.timeout.sec = int(self.timeout_sec)
        request.ik_request.timeout.nanosec = int(
            (self.timeout_sec - int(self.timeout_sec)) * 1e9
        )

        p = pose_msg.pose.position
        q = pose_msg.pose.orientation

        self.get_logger().info(
            f"Checking IK for {self.ik_link_name} in frame {pose_msg.header.frame_id}: "
            f"position=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), "
            f"orientation=({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f})"
        )

        future = self.ik_client.call_async(request)
        future.add_done_callback(self.ik_result_callback)

    def ik_result_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"IK service call failed: {e}")
            self.busy = False
            return

        error_code = response.error_code.val

        if error_code == 1:
            self.get_logger().info("IK SUCCESS: pose is solvable.")

            names = response.solution.joint_state.name
            positions = response.solution.joint_state.position

            self.get_logger().info("IK joint solution:")
            for name, pos in zip(names, positions):
                self.get_logger().info(f"  {name}: {pos:.4f} rad")

        else:
            self.get_logger().error(f"IK FAILED: error_code={error_code}")
            self.get_logger().error(
                "Try checking: pose reachability, orientation quaternion, collision state, "
                "planning_group, or ik_link_name."
            )

        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = IKChecker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
