#!/usr/bin/env python3

from collections import deque
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import String

from yolo_msgs.msg import DetectionArray


class ObjectPoseEstimator(Node):
    """
    YOLO bbox + aligned depth + camera_info
    -> raw 3D object pose
    -> filtered stable 3D object pose for grasp planning
    """

    def __init__(self):
        super().__init__("object_pose_estimator")

        # Main parameters
        self.declare_parameter("target_class", "orange")
        self.declare_parameter("min_confidence", 0.4)
        self.declare_parameter("camera_frame", "camera_color_optical_frame")

        # Depth parameters
        self.declare_parameter("min_depth_m", 0.10)
        self.declare_parameter("max_depth_m", 1.20)
        self.declare_parameter("bbox_depth_scale", 0.4)

        # Temporal filter parameters
        self.declare_parameter("buffer_size", 10)
        self.declare_parameter("min_stable_count", 5)
        self.declare_parameter("max_position_std_m", 0.025)

        self.target_class = self.get_parameter("target_class").value
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.camera_frame = self.get_parameter("camera_frame").value

        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.bbox_depth_scale = float(self.get_parameter("bbox_depth_scale").value)

        self.buffer_size = int(self.get_parameter("buffer_size").value)
        self.min_stable_count = int(self.get_parameter("min_stable_count").value)
        self.max_position_std_m = float(self.get_parameter("max_position_std_m").value)

        # Store detected object class here
        self.current_object_class: Optional[str] = None

        # Buffer of recent valid object positions in camera frame
        self.pose_buffer = deque(maxlen=self.buffer_size)

        self.bridge = CvBridge()

        self.latest_depth_image: Optional[np.ndarray] = None
        self.latest_depth_encoding: Optional[str] = None

        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None

        # Subscribers
        self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self.depth_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            DetectionArray,
            "/yolo/detections",
            self.detections_callback,
            10,
        )

        # Publishers
        self.raw_pose_pub = self.create_publisher(
            PoseStamped,
            "/object_pose_camera_raw",
            10,
        )

        self.filtered_pose_pub = self.create_publisher(
            PoseStamped,
            "/object_pose_camera",
            10,
        )

        self.class_pub = self.create_publisher(
            String,
            "/selected_object_class",
            10,
        )

        self.marker_pub = self.create_publisher(
            Marker,
            "/object_pose_camera_marker",
            10,
        )

        self.get_logger().info("Object pose estimator with temporal filter started.")
        self.get_logger().info(f"Target class: {self.target_class}")
        self.get_logger().info(f"Camera frame: {self.camera_frame}")

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth_encoding = msg.encoding
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            )
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def detections_callback(self, msg: DetectionArray):
        if self.latest_depth_image is None:
            self.get_logger().warn("No aligned depth image received yet.")
            return

        if None in [self.fx, self.fy, self.cx, self.cy]:
            self.get_logger().warn("No camera_info received yet.")
            return

        detection = self.select_detection(msg)
        if detection is None:
            return

        self.current_object_class = detection.class_name

        u = float(detection.bbox.center.position.x)
        v = float(detection.bbox.center.position.y)
        bbox_w = float(detection.bbox.size.x)
        bbox_h = float(detection.bbox.size.y)

        depth_m = self.get_bbox_median_depth(u, v, bbox_w, bbox_h)

        if depth_m is None:
            self.get_logger().warn(
                f"No valid depth for {detection.class_name} around bbox center "
                f"u={u:.1f}, v={v:.1f}"
            )
            return

        x, y, z = self.pixel_to_camera_xyz(u, v, depth_m)

        raw_pose = self.make_pose_msg(x, y, z, msg.header.stamp)
        self.raw_pose_pub.publish(raw_pose)

        self.pose_buffer.append((x, y, z))

        stable_pose = self.get_stable_pose(msg.header.stamp)
        if stable_pose is None:
            self.get_logger().info(
                f"Collecting stable observations for {detection.class_name}: "
                f"{len(self.pose_buffer)}/{self.min_stable_count}"
            )
            return

        self.filtered_pose_pub.publish(stable_pose)
        self.publish_marker(stable_pose)

        class_msg = String()
        class_msg.data = self.current_object_class
        self.class_pub.publish(class_msg)

        self.get_logger().info(
            f"STABLE {self.current_object_class}: "
            f"raw_xyz=({x:.3f}, {y:.3f}, {z:.3f}) "
            f"filtered_xyz=({stable_pose.pose.position.x:.3f}, "
            f"{stable_pose.pose.position.y:.3f}, "
            f"{stable_pose.pose.position.z:.3f})"
        )

    def select_detection(self, msg: DetectionArray):
        candidates = []

        for det in msg.detections:
            if det.score < self.min_confidence:
                continue

            if self.target_class and det.class_name != self.target_class:
                continue

            candidates.append(det)

        if not candidates:
            return None

        # Minimal policy: choose the highest-confidence matching object.
        candidates.sort(key=lambda d: d.score, reverse=True)
        return candidates[0]

    def get_bbox_median_depth(
        self,
        u: float,
        v: float,
        bbox_w: float,
        bbox_h: float,
    ) -> Optional[float]:
        """
        Use the central region of the bbox, not just one pixel.
        This is more stable for curved or noisy objects like oranges.
        """
        h, w = self.latest_depth_image.shape[:2]

        region_w = max(5, int(bbox_w * self.bbox_depth_scale))
        region_h = max(5, int(bbox_h * self.bbox_depth_scale))

        u_i = int(round(u))
        v_i = int(round(v))

        u_min = max(0, u_i - region_w // 2)
        u_max = min(w, u_i + region_w // 2 + 1)
        v_min = max(0, v_i - region_h // 2)
        v_max = min(h, v_i + region_h // 2 + 1)

        if u_min >= u_max or v_min >= v_max:
            return None

        patch = self.latest_depth_image[v_min:v_max, u_min:u_max].astype(np.float32)

        if self.latest_depth_encoding == "16UC1":
            patch_m = patch / 1000.0
        elif self.latest_depth_encoding == "32FC1":
            patch_m = patch
        else:
            # Fallback heuristic
            if np.nanmedian(patch) > 10.0:
                patch_m = patch / 1000.0
            else:
                patch_m = patch

        valid = patch_m[np.isfinite(patch_m)]
        valid = valid[valid > 0.0]
        valid = valid[(valid >= self.min_depth_m) & (valid <= self.max_depth_m)]

        if valid.size == 0:
            return None

        # Remove depth outliers using percentile trimming.
        low = np.percentile(valid, 20)
        high = np.percentile(valid, 80)
        trimmed = valid[(valid >= low) & (valid <= high)]

        if trimmed.size == 0:
            return float(np.median(valid))

        return float(np.median(trimmed))

    def pixel_to_camera_xyz(self, u: float, v: float, z: float) -> Tuple[float, float, float]:
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return x, y, z

    def get_stable_pose(self, stamp) -> Optional[PoseStamped]:
        if len(self.pose_buffer) < self.min_stable_count:
            return None

        points = np.array(self.pose_buffer, dtype=np.float32)

        std_xyz = np.std(points, axis=0)
        max_std = float(np.max(std_xyz))

        if max_std > self.max_position_std_m:
            self.get_logger().info(
                f"Pose not stable yet. std={max_std:.3f} m"
            )
            return None

        median_xyz = np.median(points, axis=0)

        return self.make_pose_msg(
            float(median_xyz[0]),
            float(median_xyz[1]),
            float(median_xyz[2]),
            stamp,
        )

    def make_pose_msg(self, x: float, y: float, z: float, stamp) -> PoseStamped:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.camera_frame

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z

        # Orientation is unknown for now.
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        return pose_msg

    def publish_marker(self, pose_msg: PoseStamped):
        marker = Marker()
        marker.header = pose_msg.header
        marker.ns = "filtered_object_pose_camera"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose = pose_msg.pose

        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04

        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 1.0

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPoseEstimator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()