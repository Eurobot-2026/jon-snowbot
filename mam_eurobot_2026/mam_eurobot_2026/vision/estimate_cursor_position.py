#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
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

        w = int(self.get_parameter("image_width").value)
        h = int(self.get_parameter("image_height").value)
        hfov = float(self.get_parameter("horizontal_fov_deg").value)
        self.fx, self.fy, self.cx, self.cy = compute_pinhole_params(w, h, hfov)
        self.cursor_height = float(self.get_parameter("cursor_height").value)
        self.cursor_world_x = float(self.get_parameter("cursor_world_x").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.tf_timeout = Duration(seconds=float(self.get_parameter("tf_timeout_sec").value))

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
