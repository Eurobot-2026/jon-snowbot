#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rclpy.time import Time

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, TransformStamped
from sensor_msgs.msg import Image
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
    TransformBroadcaster,
)
import tf_transformations as tft

"""
Estimate the world coordinate of the detected cursor.
Inputs:
  - /camera/features/cursor_center (geometry_msgs/PointStamped, pixel x/y in image frame)
  - TF transform broadcast by world_to_topcamera.py (world -> observation_device_optical_frame)
Assumptions:
  - Cursor lies on the line defined by world-frame x=cursor_world_x (default -1.0 m) and z=cursor_height (default 0.08 m)
  - Camera intrinsics follow the parameters declared in this node.
Output:
  - /world_coordinate/cursor (geometry_msgs/PointStamped) reporting cursor position in world frame.
  - TF world -> cursor_frame (translation-only; cursor frame shares world's orientation).
  - OpenCV window that overlays the cursor frame axes on the incoming image.
"""


def compute_pinhole_params(width: int, height: int, horizontal_fov_deg: float) -> Tuple[float, float, float, float]:
    """Return fx, fy, cx, cy from image size and horizontal FOV (fx==fy assumption)."""
    hfov_rad = math.radians(horizontal_fov_deg)
    fx = (width / 2.0) / math.tan(hfov_rad / 2.0)
    fy = fx
    cx = width / 2.0
    cy = height / 2.0
    return fx, fy, cx, cy


class CursorPositionEstimator(Node):
    def __init__(self):
        super().__init__("estimate_cursor_position")

        # Parameters related to input/output and geometry assumptions
        self.declare_parameter("input_topic", "/camera/features/cursor_center")
        self.declare_parameter("output_topic", "/world_coordinate/cursor")
        self.declare_parameter("camera_frame", "observation_device_optical_frame")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("cursor_height", 0.08)
        self.declare_parameter("cursor_world_x", -1.0)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("horizontal_fov_deg", 60.0)
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("cursor_frame", "cursor_frame")
        self.declare_parameter("cursor_axis_length_m", 0.1)
        self.declare_parameter("viz_image_topic", "/top_camera/image_3")

        w = int(self.get_parameter("image_width").value)
        h = int(self.get_parameter("image_height").value)
        hfov = float(self.get_parameter("horizontal_fov_deg").value)
        self.fx, self.fy, self.cx, self.cy = compute_pinhole_params(w, h, hfov)
        self.cursor_height = float(self.get_parameter("cursor_height").value)
        self.cursor_world_x = float(self.get_parameter("cursor_world_x").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.cursor_frame = str(self.get_parameter("cursor_frame").value)
        self.tf_timeout = Duration(seconds=float(self.get_parameter("tf_timeout_sec").value))
        self.cursor_axis_len = float(self.get_parameter("cursor_axis_length_m").value)
        self.viz_image_topic = str(self.get_parameter("viz_image_topic").value or "")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.publisher = self.create_publisher(PointStamped, self.get_parameter("output_topic").value, qos)
        self.subscription = self.create_subscription(
            PointStamped,
            self.get_parameter("input_topic").value,
            self._cursor_callback,
            qos,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.bridge: Optional[CvBridge] = None
        self.image_sub = None
        self.window_name = "Cursor frame overlay"
        if self.viz_image_topic:
            self.bridge = CvBridge()
            image_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
            self.image_sub = self.create_subscription(Image, self.viz_image_topic, self._image_callback, image_qos)
            self.get_logger().info(f"Visualization enabled; subscribing to {self.viz_image_topic}")
        else:
            self.get_logger().info("Visualization disabled (viz_image_topic is empty).")

        # Cache the latest cursor pose and transform for visualization.
        self.last_cursor_world: Optional[np.ndarray] = None
        self.last_cursor_stamp = None
        self.last_R_wc: Optional[np.ndarray] = None
        self.last_t_wc: Optional[np.ndarray] = None
        self.get_logger().info(
            f"Awaiting cursor pixels on {self.get_parameter('input_topic').value}. "
            f"Publishing world points to {self.get_parameter('output_topic').value}."
        )

    def _lookup_transform(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Fetch the latest world->camera transform, return (R_wc, t_wc)."""
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.camera_frame,
                Time(),
                timeout=self.tf_timeout,
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f"TF lookup failed for {self.world_frame}->{self.camera_frame}: {exc}"
            )
            return None

        trans = tf_msg.transform.translation
        rot = tf_msg.transform.rotation
        t_wc = np.array([trans.x, trans.y, trans.z], dtype=np.float64)
        R_wc = tft.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])[:3, :3]
        return R_wc, t_wc

    def _pixel_to_world(self, u: float, v: float, R_wc: np.ndarray, t_wc: np.ndarray) -> Optional[np.ndarray]:
        """Project pixel (u,v) to world coordinate with fixed x=cursor_world_x and z=cursor_height."""
        direction_cam = np.array([(u - self.cx) / self.fx,
                                  (v - self.cy) / self.fy,
                                  1.0], dtype=np.float64)
        direction_world = R_wc @ direction_cam

        # Solve for the scale that best satisfies x=cursor_world_x and z=cursor_height.
        dir_xz = direction_world[[0, 2]]
        offset_xz = np.array(
            [self.cursor_world_x - t_wc[0], self.cursor_height - t_wc[2]],
            dtype=np.float64,
        )

        denom = np.dot(dir_xz, dir_xz)
        if denom < 1e-8:
            self.get_logger().warn("Projection ray nearly parallel to constrained axes; skipping cursor sample.")
            return None

        scale = float(np.dot(dir_xz, offset_xz) / denom)
        if scale <= 0.0:
            self.get_logger().warn("Computed intersection behind camera or on camera plane; ignoring sample.")
            return None

        world_point = t_wc + direction_world * scale
        # Enforce the known constraints explicitly to avoid numerical drift.
        world_point[0] = self.cursor_world_x
        world_point[2] = self.cursor_height
        return world_point

    def _publish_cursor_tf(self, world_point: np.ndarray, stamp):
        """Broadcast world -> cursor_frame as a pure translation (orientation matches world)."""
        tf_msg = TransformStamped()
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id = self.cursor_frame
        tf_msg.header.stamp = stamp
        tf_msg.transform.translation.x = float(world_point[0])
        tf_msg.transform.translation.y = float(world_point[1])
        tf_msg.transform.translation.z = float(world_point[2])
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_msg)

    def _project_world_points(self, points_w: np.ndarray, R_wc: np.ndarray, t_wc: np.ndarray) -> Optional[np.ndarray]:
        """Project world points to pixel coordinates; returns Nx2 array or None on invalid depth."""
        R_cw = R_wc.T
        pixels = []
        for p_w in points_w:
            p_c = R_cw @ (p_w - t_wc)
            if p_c[2] <= 1e-6:
                return None
            u = (p_c[0] / p_c[2]) * self.fx + self.cx
            v = (p_c[1] / p_c[2]) * self.fy + self.cy
            pixels.append((u, v))
        return np.array(pixels, dtype=np.float64)

    def _draw_cursor_axes(self, frame: np.ndarray) -> Optional[np.ndarray]:
        """Overlay cursor axes on the frame using the last cursor pose and transform."""
        if self.last_cursor_world is None or self.last_R_wc is None or self.last_t_wc is None:
            return None

        origin = self.last_cursor_world
        axes_world = np.array([
            origin,
            origin + np.array([self.cursor_axis_len, 0.0, 0.0]),
            origin + np.array([0.0, self.cursor_axis_len, 0.0]),
            origin + np.array([0.0, 0.0, self.cursor_axis_len]),
        ], dtype=np.float64)

        pixels = self._project_world_points(axes_world, self.last_R_wc, self.last_t_wc)
        if pixels is None:
            return None

        overlay = frame.copy()
        origin_px = tuple(np.round(pixels[0]).astype(int))
        colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]  # x, y, z
        for idx, color in enumerate(colors, start=1):
            end_px = tuple(np.round(pixels[idx]).astype(int))
            cv2.line(overlay, origin_px, end_px, color, 2, cv2.LINE_AA)
        cv2.circle(overlay, origin_px, 4, (255, 255, 255), -1)
        cv2.putText(
            overlay,
            f"{self.cursor_frame} (world-aligned)",
            (origin_px[0] + 5, origin_px[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        return overlay

    def _image_callback(self, msg: Image):
        if self.bridge is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert image for visualization: {exc}")
            return

        if self.last_cursor_world is None:
            return

        # Use the latest known transform from the cursor callback.
        overlay = self._draw_cursor_axes(frame)
        if overlay is None:
            return

        cv2.imshow(self.window_name, overlay)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("Quit visualization requested (q).")
            rclpy.shutdown()

    def _cursor_callback(self, msg: PointStamped):
        transform = self._lookup_transform()
        if transform is None:
            return
        R_wc, t_wc = transform
        world_point = self._pixel_to_world(msg.point.x, msg.point.y, R_wc, t_wc)
        if world_point is None:
            return

        out = PointStamped()
        out.header.frame_id = self.world_frame
        out.header.stamp = msg.header.stamp if msg.header.stamp.sec or msg.header.stamp.nanosec else self.get_clock().now().to_msg()
        out.point.x = float(world_point[0])
        out.point.y = float(world_point[1])
        out.point.z = float(world_point[2])
        self.get_logger().info(f"cursor point in world; x:{world_point[0]}, y:{world_point[1]}, z:{world_point[2]}")
        self.publisher.publish(out)
        self._publish_cursor_tf(world_point, out.header.stamp)

        # Cache for visualization
        self.last_cursor_world = world_point
        self.last_cursor_stamp = out.header.stamp
        self.last_R_wc = R_wc
        self.last_t_wc = t_wc


def main():
    rclpy.init()
    node = CursorPositionEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
