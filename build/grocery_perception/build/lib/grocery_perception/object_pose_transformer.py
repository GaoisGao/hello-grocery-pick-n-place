#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose_stamped


class ObjectPoseTransformer(Node):
    def __init__(self):
        super().__init__("object_pose_transformer")

        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("input_topic", "/object_pose_camera")
        self.declare_parameter("output_topic", "/object_pose_base")

        self.target_frame = self.get_parameter("target_frame").value
        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.input_topic,
            self.pose_callback,
            10,
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.output_topic,
            10,
        )

        self.marker_pub = self.create_publisher(
            Marker,
            "/object_pose_base_marker",
            10,
        )

        self.get_logger().info("Object pose transformer started.")
        self.get_logger().info(f"Input topic: {self.input_topic}")
        self.get_logger().info(f"Output topic: {self.output_topic}")
        self.get_logger().info(f"Target frame: {self.target_frame}")

    def pose_callback(self, pose_msg: PoseStamped):
        source_frame = pose_msg.header.frame_id

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )

            transformed_pose = do_transform_pose_stamped(pose_msg, transform)
            transformed_pose.header.frame_id = self.target_frame
            transformed_pose.header.stamp = self.get_clock().now().to_msg()

            self.pose_pub.publish(transformed_pose)
            self.publish_marker(transformed_pose)

            p = transformed_pose.pose.position
            self.get_logger().info(
                f"Object in {self.target_frame}: "
                f"x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}"
            )

        except TransformException as ex:
            self.get_logger().warn(
                f"Could not transform from {source_frame} to {self.target_frame}: {ex}"
            )

    def publish_marker(self, pose_msg: PoseStamped):
        marker = Marker()
        marker.header = pose_msg.header
        marker.ns = "object_pose_base"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose = pose_msg.pose
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.1
        marker.color.a = 1.0

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPoseTransformer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
