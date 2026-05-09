"""
overhead_camera_localizer.py
============================
Stub node for the fixed overhead camera.

PURPOSE
-------
This node subscribes to the overhead camera feeds mounted above the table
and publishes the estimated 3D location of the apple in world coordinates.

The computer vision logic is intentionally left unimplemented — the ROS 2
plumbing, topic wiring, and tf2 transform setup are all in place. The sections
marked with # TODO are where the CV implementation belongs.

SUBSCRIBED TOPICS
-----------------
  /overhead_camera/image_raw          (sensor_msgs/msg/Image)
      A 640x480 RGB color image captured 30 times per second from the camera
      mounted 1.2 m above the table, pointing straight down. Each pixel is
      3 bytes: R, G, B, each in the range 0–255. Because this camera is
      fixed in the world, its view never changes — this makes coordinate
      frame math significantly simpler than the arm camera.

  /overhead_depth_camera/depth_image  (sensor_msgs/msg/Image)
      A 640x480 depth image captured 30 times per second. Each pixel is a
      32-bit float representing the distance from the camera to that point
      in the scene, measured in meters. Since the camera points straight
      down from a fixed height, depth values close to 1.2 m correspond to
      the table surface, and smaller values indicate objects sitting on it.

  /overhead_depth_camera/camera_info  (sensor_msgs/msg/CameraInfo)
      Metadata about the depth camera: focal length, principal point, and
      distortion coefficients. These are the "intrinsics" needed to convert
      a (pixel_x, pixel_y, depth) tuple into a 3D (x, y, z) point in the
      camera's coordinate frame. See image_geometry.PinholeCameraModel.

PUBLISHED TOPICS
----------------
  /overhead_camera/apple_location     (geometry_msgs/msg/PointStamped)
      The estimated 3D position of the apple in the WORLD coordinate frame.
      A PointStamped contains:
          header.frame_id — the coordinate frame ("world")
          header.stamp    — the time this estimate was made
          point.x/y/z     — position in meters from the world origin

      Currently publishes a dummy (0, 0, 0) placeholder. Replace with real
      detection output once the CV logic is implemented.

COORDINATE FRAMES AND TF2
--------------------------
Because the overhead camera is fixed in the world and never moves, the
transform from its frame to the world frame is constant. This means:

  - tf2 only needs to look up the transform once (it will not change)
  - X and Y position from an overhead view map almost directly to world X/Y
  - Depth from a straight-down camera gives world Z directly

This is considerably simpler than the arm camera, where the transform
changes with every joint movement. A reasonable starting approach is to
use the overhead camera for X/Y localization and the depth image for Z.

The frame_id of this camera in the transform tree is 'overhead_camera_link'.
The transform from overhead_camera_link to world can be retrieved with:

    transform = tf_buffer.lookup_transform(
        'world',
        'overhead_camera_link',
        rclpy.time.Time()
    )

SUGGESTED CV PIPELINE
----------------------
The following is a general outline of how to go from a camera frame to a
published apple location. This is not prescriptive — adapt it to whichever
detection approach makes sense for the project.

  1. Detect the apple in the RGB image
     An overhead view is well suited to color-based segmentation — looking
     for a red circular region against the brown table surface. The useful
     output is a pixel coordinate: (center_x, center_y) within the 640x480
     frame.

  2. Look up the depth at that pixel
     Index into the depth image at (center_y, center_x) — note that image
     arrays are indexed [row, col] which corresponds to [y, x]. Since the
     camera points straight down, this depth value is approximately the
     height of the apple above the ground.

  3. Convert pixel + depth to a 3D point in camera frame
     Use the camera intrinsics from /overhead_depth_camera/camera_info.
     image_geometry.PinholeCameraModel provides a clean way to do this
     without manual matrix math.

  4. Transform the point from camera frame to world frame
     Use the _lookup_camera_to_world() helper and tf2_geometry_msgs.
     Since this camera is fixed, the transform will be the same every call.

  5. Publish the result
     Replace the dummy publish in _publish_dummy_location() with the real
     world-frame PointStamped.

NOTE ON OVERHEAD VS ARM CAMERA
-------------------------------
Both this node and arm_camera_localizer publish an apple location estimate.
They are independent — either can be used, or both can be fused together
for a more robust result. The overhead camera tends to give better X/Y
accuracy; the arm camera can give better Z accuracy when the arm is close
to the apple.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener


class OverheadCameraLocalizer(Node):

    def __init__(self):
        super().__init__('overhead_camera_localizer')

        # --- tf2 setup ---
        # The Buffer stores the live transform tree that robot_state_publisher
        # broadcasts onto /tf. The TransformListener populates it in the
        # background — it does not need to be called directly.
        # Since this camera is fixed, the transform to world is constant and
        # only needs to be resolved once.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # --- Subscriptions ---

        # RGB image from the overhead camera — the primary input for detection
        self._rgb_sub = self.create_subscription(
            Image,
            '/overhead_camera/image_raw',
            self._rgb_callback,
            10,
        )

        # Depth image — provides per-pixel distance for localization
        self._depth_sub = self.create_subscription(
            Image,
            '/overhead_depth_camera/depth_image',
            self._depth_callback,
            10,
        )

        # Camera intrinsics — required to convert pixel coordinates to 3D rays
        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            '/overhead_depth_camera/camera_info',
            self._camera_info_callback,
            10,
        )

        # --- Publisher ---

        # The estimated apple location in world coordinates.
        # The manipulation node consumes this topic to plan the pick motion.
        self._apple_location_pub = self.create_publisher(
            PointStamped,
            '/overhead_camera/apple_location',
            10,
        )

        # Store the latest camera info so it is available during depth processing
        self._latest_camera_info = None

        self.get_logger().info('overhead_camera_localizer started — waiting for camera topics...')

    # ------------------------------------------------------------------ #
    #  RGB IMAGE CALLBACK                                                  #
    # ------------------------------------------------------------------ #

    def _rgb_callback(self, msg: Image):
        """
        Called every time a new color frame arrives (~30 Hz).

        msg.data      — raw pixel bytes (R, G, B, R, G, B, ...)
        msg.width     — image width in pixels (640)
        msg.height    — image height in pixels (480)
        msg.encoding  — pixel format, e.g. 'rgb8'
        msg.header    — contains frame_id ('overhead_camera_link') and timestamp

        The overhead view looks straight down at the table. The table surface
        appears as a brown rectangle; the apple appears as a red circle.
        X in the image corresponds roughly to world Y, and Y in the image
        corresponds roughly to world X — verify this in the Gazebo GUI.
        """

        # --- Diagnostic logging ---
        # Convert raw bytes to a numpy array shaped (height, width, 3).
        # The three channels are R, G, B in that order.
        pixels = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )
        mean_r = float(np.mean(pixels[:, :, 0]))
        mean_g = float(np.mean(pixels[:, :, 1]))
        mean_b = float(np.mean(pixels[:, :, 2]))
        self.get_logger().info(
            f'[overhead RGB] {msg.width}x{msg.height} | '
            f'mean R={mean_r:.1f}  G={mean_g:.1f}  B={mean_b:.1f}',
            throttle_duration_sec=2.0,
        )

        # TODO — Detection
        #   This is where the CV implementation begins. The `pixels` array
        #   above is a standard numpy image ready for processing.
        #
        #   The goal is to produce (center_x, center_y) in pixel coordinates.
        #   An overhead view of a red apple on a brown table is a good candidate
        #   for simple color segmentation before investing in a heavier approach.
        #
        #   If nothing is detected, consider returning early rather than
        #   publishing a misleading location.

        # --- Placeholder ---
        self._publish_dummy_location(msg.header.stamp)

    # ------------------------------------------------------------------ #
    #  DEPTH IMAGE CALLBACK                                                #
    # ------------------------------------------------------------------ #

    def _depth_callback(self, msg: Image):
        """
        Called every time a new depth frame arrives (~30 Hz).

        msg.data      — raw float32 bytes, one 4-byte float per pixel
        msg.width     — image width in pixels (640)
        msg.height    — image height in pixels (480)
        msg.encoding  — '32FC1' (single-channel 32-bit float)
        msg.header    — contains frame_id ('overhead_camera_link') and timestamp

        Since the camera points straight down from a fixed height of 1.2 m,
        the table surface will appear at approximately 1.2 m depth. Objects
        sitting on the table (like the apple) will appear at a smaller depth
        value — roughly 1.2 m minus the object's height.
        """

        # --- Diagnostic logging ---
        depth_pixels = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width
        )
        # NaN and inf represent pixels with no valid depth reading
        valid = depth_pixels[np.isfinite(depth_pixels)]
        if valid.size > 0:
            self.get_logger().info(
                f'[overhead depth] {msg.width}x{msg.height} | '
                f'depth min={valid.min():.2f}m  max={valid.max():.2f}m  '
                f'mean={valid.mean():.2f}m  stamp={msg.header.stamp.sec}',
                throttle_duration_sec=2.0,
            )

        # TODO — Depth lookup
        #   Once a detection pixel (center_x, center_y) is available from
        #   the RGB callback, retrieve its depth here:
        #     depth_value = depth_pixels[center_y, center_x]
        #   Note the [y, x] indexing — rows come first in numpy arrays.
        #   Verify the value is finite before using it downstream.

    # ------------------------------------------------------------------ #
    #  CAMERA INFO CALLBACK                                                #
    # ------------------------------------------------------------------ #

    def _camera_info_callback(self, msg: CameraInfo):
        """
        Called when camera intrinsics are published (typically once at startup).

        Stores the message so it is available when converting a detected pixel
        and its depth into a 3D point in camera space.

        The key field is msg.k — a 3x3 intrinsic matrix flattened to 9 values
        containing the focal lengths (fx, fy) and principal point (cx, cy).
        """
        self._latest_camera_info = msg

        # TODO — Pixel-to-3D projection
        #   With a (center_x, center_y) and a depth_value, the intrinsics
        #   here allow computing a 3D point in the camera coordinate frame.
        #   image_geometry.PinholeCameraModel is a convenient abstraction:
        #
        #     from image_geometry import PinholeCameraModel
        #     model = PinholeCameraModel()
        #     model.fromCameraInfo(self._latest_camera_info)
        #     ray = model.projectPixelTo3dRay((center_x, center_y))
        #     point_in_camera_frame = [r * depth_value for r in ray]
        #
        #   point_in_camera_frame is (x, y, z) in meters, expressed in the
        #   overhead_camera_link frame. It still needs to be transformed to
        #   world frame — though since this camera is fixed, that transform
        #   is constant.

    # ------------------------------------------------------------------ #
    #  TF2 TRANSFORM HELPER                                                #
    # ------------------------------------------------------------------ #

    def _lookup_camera_to_world(self):
        """
        Returns the transform from overhead_camera_link to world, or None
        if it is not yet available (e.g. during early startup).

        Since this camera is fixed in the world, this transform is constant
        and only needs to be looked up once — it will not change over time.

        Once a 3D point in overhead_camera_link frame is known, this transform
        can be applied to express it in the world frame:

            import tf2_geometry_msgs
            transform = self._lookup_camera_to_world()
            if transform is None:
                return  # not ready — skip this frame

            point_stamped = PointStamped()
            point_stamped.header.frame_id = 'overhead_camera_link'
            point_stamped.point = Point(x=..., y=..., z=...)
            world_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            # world_point.point is now (x, y, z) in world frame
        """
        try:
            return self._tf_buffer.lookup_transform(
                'world',
                'overhead_camera_link',
                rclpy.time.Time(),
            )
        except Exception:
            # Transform not available yet — will resolve shortly after startup
            return None

    # ------------------------------------------------------------------ #
    #  PLACEHOLDER PUBLISHER                                               #
    # ------------------------------------------------------------------ #

    def _publish_dummy_location(self, stamp):
        """
        Publishes a dummy (0, 0, 0) apple location so the output topic is
        always active and downstream nodes do not block waiting for it.

        TODO — Replace this with the real world-frame PointStamped once the
        detection and transform pipeline above is complete, then remove this
        method.
        """
        self.get_logger().warn(
            'CV NOT IMPLEMENTED — publishing dummy apple location (0, 0, 0)',
            throttle_duration_sec=5.0,
        )
        msg = PointStamped(
            header=Header(frame_id='world', stamp=stamp),
            point=Point(x=0.0, y=0.0, z=0.0),
        )
        self._apple_location_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OverheadCameraLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
