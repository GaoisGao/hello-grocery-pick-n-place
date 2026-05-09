#!/usr/bin/env python3

from typing import Tuple

import rclpy
from rclpy.node import Node

from collections import deque
import math

from geometry_msgs.msg import PoseStamped

from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class PickPlanner(Node):

    """
    Converts object center pose in base_link into a simple top-down pick sequence.

    Input:
      /object_pose_base : PoseStamped

    Outputs:
      /pre_grasp_pose : PoseStamped
      /grasp_pose     : PoseStamped
      /lift_pose      : PoseStamped

    Assumption:
      The grasp is top-down. The gripper/camera is facing downward toward the object.
    """

    def __init__(self):
        super().__init__("pick_planner")

        # ---- Tunable position offsets ----
        # These are relative to the detected object center.
        # You must tune these experimentally for your gripper geometry.
        self.declare_parameter("pre_grasp_z_offset", 0.12)   # meters above object center
        self.declare_parameter("grasp_z_offset", 0.04)       # meters above object center
        self.declare_parameter("lift_z_offset", 0.22)        # meters above object center

        # Optional XY offsets.
        # Use these if your camera detects the object center, but your gripper TCP
        # needs to approach slightly offset from that center.
        self.declare_parameter("grasp_x_offset", 0.00)
        self.declare_parameter("grasp_y_offset", 0.00)

        self.declare_parameter("stable_window", 5)
        self.declare_parameter("max_position_std_m", 0.03)

        self.stable_window = int(self.get_parameter("stable_window").value)
        self.max_position_std_m = float(self.get_parameter("max_position_std_m").value)

        self.pose_buffer = deque(maxlen=self.stable_window)

        # ---- Tunable orientation ----
        # Fixed top-down gripper orientation as quaternion in base_link.
        #
        # IMPORTANT:
        # These default values are placeholders.
        # You should manually move the robot to a good top-down pre-grasp orientation,
        # then read:
        #
        #   ros2 run tf2_ros tf2_echo base_link tool0
        #
        # Copy the quaternion into these parameters.
        self.declare_parameter("grasp_qx", 0.0)
        self.declare_parameter("grasp_qy", 1.0)
        self.declare_parameter("grasp_qz", 0.0)
        self.declare_parameter("grasp_qw", 0.0)

        self.pre_grasp_z_offset = float(self.get_parameter("pre_grasp_z_offset").value)
        self.grasp_z_offset = float(self.get_parameter("grasp_z_offset").value)
        self.lift_z_offset = float(self.get_parameter("lift_z_offset").value)

        self.grasp_x_offset = float(self.get_parameter("grasp_x_offset").value)
        self.grasp_y_offset = float(self.get_parameter("grasp_y_offset").value)

        self.grasp_quat = (
            float(self.get_parameter("grasp_qx").value),
            float(self.get_parameter("grasp_qy").value),
            float(self.get_parameter("grasp_qz").value),
            float(self.get_parameter("grasp_qw").value),
        )

        self.object_sub = self.create_subscription(
            PoseStamped,
            "/object_pose_base",
            self.object_pose_callback,
            10,
        )

        self.pre_grasp_pub = self.create_publisher(PoseStamped, "/pre_grasp_pose", 10)
        self.grasp_pub = self.create_publisher(PoseStamped, "/grasp_pose", 10)
        self.lift_pub = self.create_publisher(PoseStamped, "/lift_pose", 10)

        self.scene_pub = self.create_publisher(
            PlanningScene,
            "/planning_scene",
            10
        )

        self.get_logger().info("Pick planner started.")
        self.get_logger().info("Tunable parameters:")
        self.get_logger().info(f"  pre_grasp_z_offset = {self.pre_grasp_z_offset:.3f} m")
        self.get_logger().info(f"  grasp_z_offset     = {self.grasp_z_offset:.3f} m")
        self.get_logger().info(f"  lift_z_offset      = {self.lift_z_offset:.3f} m")
        self.get_logger().info(f"  grasp_x_offset     = {self.grasp_x_offset:.3f} m")
        self.get_logger().info(f"  grasp_y_offset     = {self.grasp_y_offset:.3f} m")
        self.get_logger().info(f"  grasp_quaternion   = {self.grasp_quat}")

    def object_pose_callback(self, object_pose: PoseStamped):
        if object_pose.header.frame_id != "base_link":
            self.get_logger().warn(
                f"Expected object pose in base_link, got {object_pose.header.frame_id}"
            )

        x_obj = object_pose.pose.position.x
        y_obj = object_pose.pose.position.y
        z_obj = object_pose.pose.position.z

        self.publish_detected_object_collision(object_pose)

        self.pose_buffer.append((x_obj, y_obj, z_obj))

        if len(self.pose_buffer) < self.stable_window:
            self.get_logger().info(
                f"Collecting stable object poses: {len(self.pose_buffer)}/{self.stable_window}"
            )
            return

        xs = [p[0] for p in self.pose_buffer]
        ys = [p[1] for p in self.pose_buffer]
        zs = [p[2] for p in self.pose_buffer]

        x_mean = sum(xs) / len(xs)
        y_mean = sum(ys) / len(ys)
        z_mean = sum(zs) / len(zs)

        def std(values, mean):
            return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))

        x_std = std(xs, x_mean)
        y_std = std(ys, y_mean)
        z_std = std(zs, z_mean)

        max_std = max(x_std, y_std, z_std)

        if max_std > self.max_position_std_m:
            self.get_logger().warn(
                f"Object pose not stable yet. "
                f"std=({x_std:.3f}, {y_std:.3f}, {z_std:.3f}), "
                f"max allowed={self.max_position_std_m:.3f}"
            )
            return

        x_obj = x_mean
        y_obj = y_mean
        z_obj = z_mean

        # Apply optional XY grasp correction.
        x_target = x_obj + self.grasp_x_offset
        y_target = y_obj + self.grasp_y_offset

        pre_grasp_pose = self.make_pose(
            source=object_pose,
            x=x_target,
            y=y_target,
            z=z_obj + self.pre_grasp_z_offset,
            quat=self.grasp_quat,
        )

        grasp_pose = self.make_pose(
            source=object_pose,
            x=x_target,
            y=y_target,
            z=z_obj + self.grasp_z_offset,
            quat=self.grasp_quat,
        )

        lift_pose = self.make_pose(
            source=object_pose,
            x=x_target,
            y=y_target,
            z=z_obj + self.lift_z_offset,
            quat=self.grasp_quat,
        )

        self.pre_grasp_pub.publish(pre_grasp_pose)
        self.grasp_pub.publish(grasp_pose)
        self.lift_pub.publish(lift_pose)

        self.get_logger().info(
            "Published pick poses | "
            f"object=({x_obj:.3f}, {y_obj:.3f}, {z_obj:.3f}) | "
            f"pre_grasp=({pre_grasp_pose.pose.position.x:.3f}, "
            f"{pre_grasp_pose.pose.position.y:.3f}, "
            f"{pre_grasp_pose.pose.position.z:.3f}) | "
            f"grasp=({grasp_pose.pose.position.x:.3f}, "
            f"{grasp_pose.pose.position.y:.3f}, "
            f"{grasp_pose.pose.position.z:.3f}) | "
            f"lift=({lift_pose.pose.position.x:.3f}, "
            f"{lift_pose.pose.position.y:.3f}, "
            f"{lift_pose.pose.position.z:.3f})"
        )

    def make_pose(
        self,
        source: PoseStamped,
        x: float,
        y: float,
        z: float,
        quat: Tuple[float, float, float, float],
    ) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = source.header.frame_id

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        return pose

    def publish_detected_object_collision(self, object_pose: PoseStamped):
        scene = PlanningScene()
        scene.is_diff = True

        obj = CollisionObject()
        obj.id = "detected_object"
        obj.header.frame_id = object_pose.header.frame_id

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE

        # Radius in meters.
        # Use 0.04 for a small orange/apple-sized object.
        primitive.dimensions = [0.04]

        pose = Pose()
        pose.position.x = object_pose.pose.position.x
        pose.position.y = object_pose.pose.position.y
        pose.position.z = object_pose.pose.position.z
        pose.orientation.w = 1.0

        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        scene.world.collision_objects.append(obj)

        self.scene_pub.publish(scene)

        self.get_logger().info(
            "Published detected object collision sphere | "
            f"frame={obj.header.frame_id}, "
            f"x={pose.position.x:.3f}, "
            f"y={pose.position.y:.3f}, "
            f"z={pose.position.z:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PickPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()