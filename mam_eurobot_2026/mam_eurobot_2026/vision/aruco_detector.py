#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Pose, PoseWithCovariance, Point, Quaternion, Vector3
from rclpy.node import Node
from vision_msgs.msg import (
    Detection3DArray,
    Detection3D,
    ObjectHypothesisWithPose,
    ObjectHypothesis,
    BoundingBox3D,
)
import math
import cv2
import cv2.aruco as aruco
import numpy as np
import yaml
from pathlib import Path
from importlib.resources import files
from cv_bridge import CvBridge

# this node is only for Image, does not assume CameraInfo
# input; sensor_msgs/Image　
# output; vision_msgs/Detection3DArray
# ros2 run mam_eurobot_2026 aruco_detector --ros-args -p image_topic:=<topic name>

class ArucoDetectNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.get_logger().info("Aruco Detect Node created!")

        self.declare_parameter('config_yaml', 'vision_settings.yaml')
        image_topic_param = self.declare_parameter('image_topic', value='')
        self.image_topic = image_topic_param.get_parameter_value().string_value
        if not self.image_topic:
            raise ValueError("Parameter 'image_topic' is required. Pass via --ros-args -p image_topic:=<topic>.")
        self.get_logger().info(f"Subscribing to image topic: {self.image_topic}")
        self.show_debug = bool(self.declare_parameter('show_debug_image', True).value)

        cfg = self._load_config()
        self._set_camera_parameters(cfg)

        # aruco detector
        self.parameters = aruco.DetectorParameters_create()

        self.bridge = CvBridge()

        qos = QoSProfile(depth=1)
        self.pub = self.create_publisher(Detection3DArray, '/aruco/detections', qos)
        self.create_subscription(Image, self.image_topic, self._image_callback, qos)
    
    def _load_config(self) -> dict:
        """Load vision_settings.yaml from path or package data."""
        cfg_name_or_path = self.get_parameter("config_yaml").get_parameter_value().string_value

        def _try_open_yaml(path_str: str):
            p = Path(path_str)
            if p.is_absolute() and p.exists():
                with p.open('r', encoding='utf-8') as f:
                    return yaml.safe_load(f)
            cand = Path.cwd() / p
            if cand.exists():
                with cand.open('r', encoding='utf-8') as f:
                    return yaml.safe_load(f)
            return None

        cfg = _try_open_yaml(cfg_name_or_path)
        if cfg is None:
            fname = Path(cfg_name_or_path).name if cfg_name_or_path else "vision_settings.yaml"
            path = files("mam_eurobot_2026.vision").joinpath(fname)
            if not path.is_file():
                raise FileNotFoundError(f"Config not found at '{cfg_name_or_path}' or packaged file '{fname}'")
            with path.open('r', encoding='utf-8') as f:
                cfg = yaml.safe_load(f)
            self.get_logger().info(f"Loaded config from package data: mam_eurobot_2026/vision/{fname}")
        else:
            self.get_logger().info(f"Loaded config from path: {cfg_name_or_path}")

        return cfg


    def _set_camera_parameters(self, cfg: dict):
        g = cfg.get("global", {})
        self.fov = math.radians(float(g.get("horizontal_fov_deg", 60.0)))
        self.width = float(g.get("image_width", 640))
        self.height = float(g.get("image_height", 480))
        dict_name = str(g.get("aruco_dictionary", "DICT_4X4_50"))
        self.aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, dict_name, aruco.DICT_4X4_50))

        cameras = cfg.get("cameras", [])
        cam_cfg = None
        for cam in cameras:
            if cam.get("image_topic") == self.image_topic:
                cam_cfg = cam
                break
        if cam_cfg is None and cameras:
            cam_cfg = cameras[0]
            self.get_logger().warn(
                f"No camera in config matched image_topic={self.image_topic}; using first entry '{cam_cfg.get('name')}'."
            )
        if cam_cfg is None:
            raise ValueError("No cameras defined in vision_settings.yaml; cannot configure ArUco detector.")

        # marker size map: prefer per-ID sizes (when provided), or generic aruco length, else camera size
        self.aruco_sizes, self.generic_aruco_len, self.allowed_ids = self._parse_arucos(cfg)
        self.default_marker_len = self._marker_length_from_config(cfg, cam_cfg)
        self.get_logger().info(
            f"Camera config: name={cam_cfg.get('name')} topic={cam_cfg.get('image_topic')} "
            f"marker_id={cam_cfg.get('marker_id')} marker_length_m={self.default_marker_len}"
        )
        log_sizes = []
        if self.generic_aruco_len is not None:
            log_sizes.append(f"default_arucos_length={self.generic_aruco_len}")
        if self.aruco_sizes:
            log_sizes.append(f"per_id_sizes={self.aruco_sizes}")
        if log_sizes:
            self.get_logger().info("Aruco size config: " + ", ".join(log_sizes))
        if self.allowed_ids:
            self.get_logger().info(f"Allowed ArUco IDs: {sorted(self.allowed_ids)} (others will be ignored)")

        # fx, fy calculate fx fy from hfov
        self.fx = self.width / (2.0 * math.tan(self.fov / 2.0))
        self.fy = self.fx
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0

    @staticmethod
    def _marker_length_from_config(cfg: dict, cam_cfg: dict) -> float:
        """Fallback marker length for camera pose solving."""
        return float(cam_cfg.get("marker_length_m", 0.13))

    @staticmethod
    def _parse_arucos(cfg: dict) -> tuple:
        """Return (per-id sizes, generic length, allowed_ids) from arucos section."""
        sizes = {}
        generic_len = None
        allowed = set()
        for aru in cfg.get("arucos", []):
            if "marker_length_m" not in aru:
                continue
            try:
                length = float(aru["marker_length_m"])
            except (TypeError, ValueError):
                continue
            if "id" in aru:
                try:
                    mid = int(aru["id"])
                    sizes[mid] = length
                    allowed.add(mid)
                except (TypeError, ValueError):
                    continue
            else:
                generic_len = length
        return sizes, generic_len, allowed

    def _aruco_length_for_id(self, marker_id: int) -> float:
        """Lookup marker size; fall back to default marker length from camera config."""
        if int(marker_id) in self.aruco_sizes:
            return self.aruco_sizes[int(marker_id)]
        if self.generic_aruco_len is not None:
            return self.generic_aruco_len
        return self.default_marker_len


    def _image_callback(self, img_msg: Image):
        self.get_logger().info(f"Received image frame: {img_msg.header.stamp.sec}.{img_msg.header.stamp.nanosec}")
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        det_array = self._detect_markers(img_msg, frame)
        if det_array is not None:
            self.get_logger().info(f"Publishing Detection3DArray with {len(det_array.detections)} detections")
            self.pub.publish(det_array)

    def _detect_markers(self, img_msg: Image, frame: np.ndarray):
        h, w = frame.shape[:2]

        # parameter matrix K and Distortion D
        K = np.array([[self.fx, 0, self.cx],
                      [0, self.fy, self.cy],
                      [0,     0,    1]], np.float32)
        D = np.zeros((5, 1), np.float32)

        debug_frame = frame.copy() if self.show_debug else None

        # ArUco 検出
        # corners, ids, _ = self.detector.detectMarkers(frame) # cannot be used in opencv 4.5.4
        corners, ids, _ = aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)
        if ids is not None and len(ids) > 0:
            raw_ids = ids.flatten().tolist()
            self.get_logger().info(f"Raw detected IDs: {raw_ids}")
        det_array = Detection3DArray()
        det_array.header = Header()
        det_array.header.stamp = img_msg.header.stamp
        det_array.header.frame_id = img_msg.header.frame_id or "front_camera_frame"

        if ids is None or len(ids) == 0:
            # if not detected, return empty matrix
            return det_array

        self.get_logger().info("---------- ArUco detection begin ----------")

        for corner, marker_id in zip(corners, ids.flatten()):
            if self.allowed_ids and int(marker_id) not in self.allowed_ids:
                continue
            marker_len = self._aruco_length_for_id(int(marker_id))
            rvecs, tvecs, _objpts = aruco.estimatePoseSingleMarkers(
                np.array([corner]), marker_len, K, D
            )
            rvec = rvecs[0]
            tvec = tvecs[0]

            if debug_frame is not None:
                try:
                    cv2.drawFrameAxes(debug_frame, K, D, rvec, tvec, marker_len * 0.5)
                except Exception as exc:
                    self.get_logger().warn(f"drawFrameAxes failed: {exc}")

            # rvec/tvec -> Pose
            pose = self._rvec_tvec_to_pose(rvec.reshape(3), tvec.reshape(3))

            # results: ObjectHypothesisWithPose
            hyp = ObjectHypothesis(class_id=str(int(marker_id)), score=1.0)
            pose_cov = PoseWithCovariance()
            pose_cov.pose = pose
            pose_cov.covariance = [0.0] * 36  

            owp = ObjectHypothesisWithPose(hypothesis=hyp, pose=pose_cov)

            det = Detection3D()
            det.header = det_array.header
            det.results = [owp]

            bbox = BoundingBox3D()
            bbox.center = pose
            sx = sy = float(marker_len)
            sz = 0.001
            bbox.size = Vector3(x=sx, y=sy, z=sz)
            det.bbox = bbox

            det_array.detections.append(det)
            
            # logger for debug
            self.get_logger().info(
                f"\n=== ArUco Detection ===\n"
                f"Marker ID: {marker_id}\n"
                f"rvec: {rvec}\n"
                f"tvec (camera frame, meters): {tvec}\n"
                f"Quaternion: (x={pose.orientation.x:.4f}, y={pose.orientation.y:.4f}, z={pose.orientation.z:.4f}, w={pose.orientation.w:.4f})\n"
            )
        self.get_logger().info("---------- ArUco detection end   ----------")

        if debug_frame is not None:
            cv2.imshow("aruco_detector", debug_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Quit requested (q) from debug window; shutting down...")
                rclpy.shutdown()

        return det_array
    
    @staticmethod
    def _rvec_tvec_to_pose(rvec: np.ndarray, tvec: np.ndarray) -> Pose:
        R, _ = cv2.Rodrigues(rvec.astype(np.float64))
        qw = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
        qx = (R[2, 1] - R[1, 2]) / (4.0 * qw) if qw != 0 else 0.0
        qy = (R[0, 2] - R[2, 0]) / (4.0 * qw) if qw != 0 else 0.0
        qz = (R[1, 0] - R[0, 1]) / (4.0 * qw) if qw != 0 else 0.0

        pose = Pose()
        pose.position = Point(x=float(tvec[0]), y=float(tvec[1]), z=float(tvec[2]))
        pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        return pose



def main():
    rclpy.init()
    node = ArucoDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()
