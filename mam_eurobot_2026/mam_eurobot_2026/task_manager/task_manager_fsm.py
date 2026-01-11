import math
import os
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, Twist
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection3DArray
import tf2_ros

import yaml


def _yaw_to_quat(yaw: float) -> Quaternion:
    half = yaw * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def _quat_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _make_pose_stamped(x: float, y: float, yaw: float, frame_id: str) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = Time().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation = _yaw_to_quat(yaw)
    return pose


def compute_pregrasp_pose(crate_pose: PoseStamped, distance_m: float) -> PoseStamped:
    yaw = _quat_to_yaw(crate_pose.pose.orientation)
    target_x = crate_pose.pose.position.x - distance_m * math.cos(yaw)
    target_y = crate_pose.pose.position.y - distance_m * math.sin(yaw)
    face_yaw = math.atan2(
        crate_pose.pose.position.y - target_y,
        crate_pose.pose.position.x - target_x,
    )
    return _make_pose_stamped(target_x, target_y, face_yaw, crate_pose.header.frame_id)


class Nav2Adapter:
    def __init__(self, node: Node, action_name: str):
        self._node = node
        self._client = ActionClient(node, NavigateToPose, action_name)
        self._goal_future = None
        self._result_future = None
        self._status = "idle"

    def reset(self) -> None:
        self._goal_future = None
        self._result_future = None
        self._status = "idle"

    def send_goal(self, pose: PoseStamped) -> bool:
        if self._status == "running":
            return True
        if not self._client.wait_for_server(timeout_sec=0.1):
            return False
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._goal_future = self._client.send_goal_async(goal)
        self._status = "running"
        return True

    def update(self) -> str:
        if self._status != "running":
            return self._status
        if self._goal_future and self._goal_future.done() and self._result_future is None:
            goal_handle = self._goal_future.result()
            if not goal_handle.accepted:
                self._status = "failed"
                return self._status
            self._result_future = goal_handle.get_result_async()
        if self._result_future and self._result_future.done():
            result = self._result_future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self._status = "succeeded"
            else:
                self._status = "failed"
        return self._status


class GripperAdapter:
    def __init__(self, node: Node):
        self._node = node
        self._warned = False

    def open_gripper(self) -> bool:
        # TODO: Replace with actual gripper API integration.
        if not self._warned:
            self._node.get_logger().warn("Gripper adapter is a stub; replace with real API.")
            self._warned = True
        return True

    def close_gripper(self) -> bool:
        # TODO: Replace with actual gripper API integration.
        if not self._warned:
            self._node.get_logger().warn("Gripper adapter is a stub; replace with real API.")
            self._warned = True
        return True

class DetectedCratesAdapter:
    MSG_TYPE = Detection3DArray

    def parse(self, msg) -> List[Tuple[str, PoseStamped]]:
        crates = []
        if hasattr(msg, "detections"):
            for det in msg.detections:
                if not det.results:
                    continue
                hyp = det.results[0].hypothesis
                crate_id = str(getattr(hyp, "class_id", ""))
                pose = det.results[0].pose.pose
                if crate_id and pose is not None:
                    pose_stamped = PoseStamped()
                    pose_stamped.header = det.header
                    pose_stamped.pose = pose
                    crates.append((crate_id, pose_stamped))
        return crates



class TaskManagerFSM(Node):
    def __init__(self):
        super().__init__("task_manager_fsm")

        self._start_time = time.monotonic()
        self._state = "INIT"
        self._state_start = time.monotonic()
        self._prev_state = None
        self._last_reason = ""

        self._global_frame = self.declare_parameter("global_frame", "map").value
        self._base_frame = self.declare_parameter("base_frame", "base_link").value
        self._pantry_yaml = self.declare_parameter("pantry_yaml", "").value
        self._nest_x = float(self.declare_parameter("nest_x", 0.0).value)
        self._nest_y = float(self.declare_parameter("nest_y", 0.0).value)
        self._nest_yaw = float(self.declare_parameter("nest_yaw", 0.0).value)
        self._pregrasp_offset = float(self.declare_parameter("pregrasp_offset", 0.25).value)

        self._state_timer_period = float(self.declare_parameter("state_timer_period", 0.1).value)
        self._align_duration = float(self.declare_parameter("align_duration", 1.5).value)
        self._align_speed = float(self.declare_parameter("align_speed", 0.05).value)
        self._backup_duration = float(self.declare_parameter("backup_duration", 1.5).value)
        self._backup_speed = float(self.declare_parameter("backup_speed", -0.08).value)
        self._gripper_wait = float(self.declare_parameter("gripper_wait", 0.5).value)
        self._place_wait = float(self.declare_parameter("place_wait", 0.5).value)
        self._verify_wait = float(self.declare_parameter("verify_wait", 0.5).value)

        self._timeout_move_cursor = float(self.declare_parameter("timeout_move_cursor", 5.0).value)
        self._timeout_select_crate = float(self.declare_parameter("timeout_select_crate", 5.0).value)
        self._timeout_nav = float(self.declare_parameter("timeout_nav", 20.0).value)
        self._timeout_align = float(self.declare_parameter("timeout_align", 5.0).value)
        self._timeout_hook = float(self.declare_parameter("timeout_hook", 5.0).value)
        self._timeout_gripper = float(self.declare_parameter("timeout_gripper", 5.0).value)
        self._timeout_verify = float(self.declare_parameter("timeout_verify", 5.0).value)
        self._timeout_return = float(self.declare_parameter("timeout_return", 30.0).value)

        self._gripper_joint = self.declare_parameter("gripper_joint", "gripper_joint").value
        self._gripper_closed_pos = float(self.declare_parameter("gripper_closed_pos", 0.0).value)
        self._gripper_closed_tol = float(self.declare_parameter("gripper_closed_tol", 0.01).value)
        self._pantry_region_xy = self.declare_parameter("pantry_region_xy", [0.2, 0.2]).value
        self._cursor_topic = self.declare_parameter("cursor_topic", "/world_coordinate/cursor").value
        self._cursor_grab_duration = float(self.declare_parameter("cursor_grab_duration", 1.0).value)
        self._cursor_grab_speed = float(self.declare_parameter("cursor_grab_speed", 0.05).value)
        self._cursor_release_duration = float(self.declare_parameter("cursor_release_duration", 1.0).value)
        self._cursor_release_speed = float(self.declare_parameter("cursor_release_speed", -0.05).value)

        self._nav_action_name = self.declare_parameter("nav_action_name", "navigate_to_pose").value
        self._nav = Nav2Adapter(self, self._nav_action_name)
        self._gripper = GripperAdapter(self)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._last_base_xy = None

        self._crates = {}
        self._current_crate_id = None
        self._current_pantry_id = None
        self._current_pregrasp = None
        self._current_place_pose = None
        self._failed_crates = set()
        self._placed_crates = set()

        self._plan: List[str] = []
        self._plan_index = 0
        self._allow_second_pick = False
        self._grasp_retry = 0
        self._nav_retry = 0
        self._cursor_pose: Optional[PoseStamped] = None
        self._cursor_goal_pose: Optional[PoseStamped] = None

        self._has_object = None
        self._gripper_joint_pos = None

        self._pantry_map = self._load_pantry_yaml(self._pantry_yaml)
        self._cursor_goal_pose = self._get_cursor_goal_pose()
        self._crates_adapter = DetectedCratesAdapter()

        self._task_state_pub = self.create_publisher(String, "/task_state", 10)
        self._task_goal_topic = self.declare_parameter("task_goal_topic", "/task_goal").value
        self._task_goal_pub = self.create_publisher(PoseStamped, self._task_goal_topic, 10)
        self._cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_subscription(Bool, "/gripper_has_object", self._on_has_object, 10)
        self.create_subscription(PointStamped, self._cursor_topic, self._on_cursor_point, 10)
        self.create_subscription(
            self._crates_adapter.MSG_TYPE,
            "/detected_crates",
            self._on_detected_crates,
            10,
        )

        self.create_timer(self._state_timer_period, self._step)

    def _load_pantry_yaml(self, param_value: str) -> Dict[str, Dict[str, float]]:
        if not param_value:
            self.get_logger().warn("pantry_yaml is empty; using default pantry poses.")
            return {
                "pantry": {"x": 0.5, "y": 0.0, "yaw": 0.0},
                "adjacent_pantry": {"x": 0.5, "y": 0.3, "yaw": 0.0},
            }
        data = None
        try:
            if os.path.isfile(param_value):
                with open(param_value, "r") as handle:
                    data = yaml.safe_load(handle)
            else:
                data = yaml.safe_load(param_value)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Failed to load pantry_yaml: {exc}")
            data = None
        return data or {}

    def _on_joint_states(self, msg: JointState) -> None:
        if self._gripper_joint in msg.name:
            idx = msg.name.index(self._gripper_joint)
            if idx < len(msg.position):
                self._gripper_joint_pos = msg.position[idx]

    def _on_has_object(self, msg: Bool) -> None:
        self._has_object = bool(msg.data)

    def _on_detected_crates(self, msg) -> None:
        crates = self._crates_adapter.parse(msg)
        for crate_id, pose in crates:
            self._crates[crate_id] = pose

    def _on_cursor_point(self, msg: PointStamped) -> None:
        if self._cursor_pose is not None:
            return
        pose = self._cursor_point_to_pose(msg)
        if pose is not None:
            self._cursor_pose = pose

    def _cursor_point_to_pose(self, msg: PointStamped) -> Optional[PoseStamped]:
        if not msg.header.frame_id:
            return None
        if msg.header.frame_id != self._global_frame:
            try:
                msg = self._tf_buffer.transform(
                    msg,
                    self._global_frame,
                    timeout=Duration(seconds=0.2),
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"Cursor TF failed {msg.header.frame_id}->{self._global_frame}: {exc}")
                return None
        yaw = 0.0
        robot_pose = self._get_robot_pose()
        if robot_pose is not None:
            yaw = robot_pose[2]
        pose = _make_pose_stamped(msg.point.x, msg.point.y, yaw, self._global_frame)
        pose.header.stamp = msg.header.stamp
        return pose

    def _get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._base_frame,
                Time(),
            )
            yaw = _quat_to_yaw(tf.transform.rotation)
            self._last_base_xy = (
                tf.transform.translation.x,
                tf.transform.translation.y,
            )
            return self._last_base_xy[0], self._last_base_xy[1], yaw
        except Exception:  # noqa: BLE001
            return None

    def _elapsed(self) -> float:
        return time.monotonic() - self._start_time

    def _state_elapsed(self) -> float:
        return time.monotonic() - self._state_start

    def _publish_state(self) -> None:
        msg = String()
        msg.data = f"{self._state}|crate={self._current_crate_id}|pantry={self._current_pantry_id}|reason={self._last_reason}"
        self._task_state_pub.publish(msg)

    def _publish_goal(self, pose: PoseStamped, label: str) -> None:
        if pose is None:
            return
        msg = PoseStamped()
        msg.header = pose.header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = pose.pose
        self._task_goal_pub.publish(msg)
        self.get_logger().info(f"Published task goal ({label}) to {self._task_goal_topic}")

    def _set_state(self, new_state: str, reason: str = "") -> None:
        if new_state != self._state:
            self.get_logger().info(
                f"State {self._state} -> {new_state} (crate={self._current_crate_id}, pantry={self._current_pantry_id}) {reason}"
            )
        self._prev_state = self._state
        self._state = new_state
        self._last_reason = reason
        self._state_start = time.monotonic()
        self._publish_state()

    def _get_robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._base_frame,
                Time(),
            )
            self._last_base_xy = (
                tf.transform.translation.x,
                tf.transform.translation.y,
            )
        except Exception:  # noqa: BLE001
            pass
        return self._last_base_xy

    def _select_nearest_crate(self) -> Optional[str]:
        robot_xy = self._get_robot_xy()
        if not robot_xy:
            self.get_logger().warn("Robot pose unavailable; cannot select nearest crate.")
            return None
        best_id = None
        best_dist = float("inf")
        for crate_id, pose in self._crates.items():
            if crate_id in self._failed_crates or crate_id in self._placed_crates:
                continue
            dx = pose.pose.position.x - robot_xy[0]
            dy = pose.pose.position.y - robot_xy[1]
            dist = dx * dx + dy * dy
            if dist < best_dist:
                best_dist = dist
                best_id = crate_id
        return best_id

    def _get_pantry_pose(self, pantry_id: str) -> Optional[PoseStamped]:
        data = self._pantry_map.get(pantry_id)
        if not data:
            self.get_logger().warn(f"Pantry '{pantry_id}' not found in pantry_yaml.")
            return None
        return _make_pose_stamped(data["x"], data["y"], data["yaw"], self._global_frame)

    def _get_cursor_goal_pose(self) -> Optional[PoseStamped]:
        data = self._pantry_map.get("cursor_goal")
        if not data:
            self.get_logger().warn("cursor_goal not found in pantry_yaml; MOVE_CURSOR will be skipped.")
            return None
        return _make_pose_stamped(data["x"], data["y"], data["yaw"], self._global_frame)

    def _check_grasp_success(self) -> bool:
        if self._has_object is True:
            return True
        if self._gripper_joint_pos is None:
            return False
        return abs(self._gripper_joint_pos - self._gripper_closed_pos) > self._gripper_closed_tol

    def _check_place_success(self) -> bool:
        if self._has_object is False:
            return True
        if self._gripper_joint_pos is None:
            return False
        return abs(self._gripper_joint_pos - self._gripper_closed_pos) <= self._gripper_closed_tol

    def _check_pantry_region(self) -> bool:
        if not self._current_crate_id or self._current_crate_id not in self._crates:
            return False
        if not self._current_pantry_id:
            return False
        pantry_pose = self._pantry_map.get(self._current_pantry_id)
        if not pantry_pose:
            return False
        try:
            region_x, region_y = self._pantry_region_xy
        except Exception:  # noqa: BLE001
            return False
        crate_pose = self._crates[self._current_crate_id]
        dx = abs(crate_pose.pose.position.x - pantry_pose["x"])
        dy = abs(crate_pose.pose.position.y - pantry_pose["y"])
        return dx <= region_x * 0.5 and dy <= region_y * 0.5

    def _send_cmd_vel(self, linear_x: float) -> None:
        twist = Twist()
        twist.linear.x = float(linear_x)
        self._cmd_vel_pub.publish(twist)

    def _nav_step(self, goal_pose: PoseStamped, timeout_sec: float) -> Optional[bool]:
        if not self._nav.send_goal(goal_pose):
            if self._state_elapsed() > timeout_sec:
                return False
            return None
        status = self._nav.update()
        if status == "succeeded":
            self._nav.reset()
            return True
        if status == "failed":
            self._nav.reset()
            return False
        if self._state_elapsed() > timeout_sec:
            self._nav.reset()
            return False
        return None

    def _step(self) -> None:
        if self._state == "DONE":
            return

        if self._state not in {"RETURN_TO_NEST", "DONE"}:
            if self._state_elapsed() > self._timeout_for_state():
                self._set_state("RETURN_TO_NEST", "state timeout")
                return

        if self._state == "INIT":
            self._set_state("MOVE_CURSOR", "start")
            return

        if self._state == "MOVE_CURSOR":
            if not self._cursor_goal_pose:
                self._set_state("NAV_TO_CURSOR", "cursor_goal missing; continue")
                return
            if self._cursor_pose is None:
                return
            self._nav_retry = 0
            self._set_state("NAV_TO_CURSOR", "cursor acquired")
            return

        if self._state == "NAV_TO_CURSOR":
            if not self._cursor_pose:
                self._set_state("RETURN_TO_NEST", "cursor pose missing")
                return
            if self._state_elapsed() < 0.1:
                self._publish_goal(self._cursor_pose, "cursor_pick")
            result = self._nav_step(self._cursor_pose, self._timeout_nav)
            if result is True:
                self._set_state("HOOK_GRAB", "arrived cursor")
                return
            if result is False:
                if self._nav_retry < 1:
                    self._nav_retry += 1
                    self._set_state("NAV_TO_CURSOR", "retry nav cursor")
                else:
                    self._set_state("RETURN_TO_NEST", "nav cursor failed")
            return

        if self._state == "HOOK_GRAB":
            if self._state_elapsed() < self._cursor_grab_duration:
                self._send_cmd_vel(self._cursor_grab_speed)
                return
            self._send_cmd_vel(0.0)
            self._nav_retry = 0
            self._set_state("NAV_CURSOR_TO_TARGET", "hook grab complete")
            return

        if self._state == "NAV_CURSOR_TO_TARGET":
            if not self._cursor_goal_pose:
                self._set_state("RETURN_TO_NEST", "cursor_goal missing")
                return
            if self._state_elapsed() < 0.1:
                self._publish_goal(self._cursor_goal_pose, "cursor_goal")
            result = self._nav_step(self._cursor_goal_pose, self._timeout_nav)
            if result is True:
                self._set_state("HOOK_RELEASE", "cursor moved")
                return
            if result is False:
                if self._nav_retry < 1:
                    self._nav_retry += 1
                    self._set_state("NAV_CURSOR_TO_TARGET", "retry nav cursor goal")
                else:
                    self._set_state("RETURN_TO_NEST", "nav cursor goal failed")
            return

        if self._state == "HOOK_RELEASE":
            if self._state_elapsed() < self._cursor_release_duration:
                self._send_cmd_vel(self._cursor_release_speed)
                return
            self._send_cmd_vel(0.0)
            elapsed = self._elapsed()
            if elapsed <= 30.0:
                self._plan = ["pantry"]
                self._allow_second_pick = True
            elif elapsed <= 40.0:
                self._plan = ["adjacent_pantry"]
                self._allow_second_pick = False
            else:
                self._plan = []
            self._plan_index = 0
            if not self._plan:
                self._set_state("RETURN_TO_NEST", "time window exceeded")
            else:
                self._set_state("SELECT_CRATE", "plan ready")
            return

        if self._state == "SELECT_CRATE":
            crate_id = self._select_nearest_crate()
            if not crate_id:
                self._set_state("RETURN_TO_NEST", "no crates available")
                return
            self._current_crate_id = crate_id
            self._current_pantry_id = self._plan[self._plan_index]
            crate_pose = self._crates[crate_id]
            self._current_pregrasp = compute_pregrasp_pose(crate_pose, self._pregrasp_offset)
            self._current_place_pose = self._get_pantry_pose(self._current_pantry_id)
            if not self._current_place_pose:
                self._set_state("RETURN_TO_NEST", "missing pantry pose")
                return
            self._nav_retry = 0
            self._grasp_retry = 0
            self._set_state("NAV_TO_PREGRASP", "target selected")
            return

        if self._state == "NAV_TO_PREGRASP":
            if self._state_elapsed() < 0.1:
                self._publish_goal(self._current_pregrasp, "pregrasp")
            result = self._nav_step(self._current_pregrasp, self._timeout_nav)
            if result is True:
                self._set_state("ALIGN_TO_CRATE", "arrived pregrasp")
                return
            if result is False:
                if self._nav_retry < 1:
                    self._nav_retry += 1
                    self._set_state("NAV_TO_PREGRASP", "retry nav pregrasp")
                else:
                    self._set_state("RETURN_TO_NEST", "nav pregrasp failed")
            return

        if self._state == "ALIGN_TO_CRATE":
            if self._state_elapsed() < self._align_duration:
                self._send_cmd_vel(self._align_speed)
            else:
                self._send_cmd_vel(0.0)
                self._set_state("GRIPPER_CLOSE", "align complete")
            return

        if self._state == "GRIPPER_CLOSE":
            if self._state_elapsed() < 0.1:
                self._gripper.close_gripper()
            if self._state_elapsed() >= self._gripper_wait:
                self._set_state("VERIFY_GRASP", "close issued")
            return

        if self._state == "VERIFY_GRASP":
            if self._state_elapsed() < self._verify_wait:
                return
            if self._check_grasp_success():
                self._nav_retry = 0
                self._set_state("NAV_TO_PLACE", "grasp success")
            else:
                self._set_state("RECOVERY", "grasp failed")
            return

        if self._state == "NAV_TO_PLACE":
            if self._state_elapsed() < 0.1:
                self._publish_goal(self._current_place_pose, "place")
            result = self._nav_step(self._current_place_pose, self._timeout_nav)
            if result is True:
                self._set_state("PLACE_RELEASE", "arrived pantry")
                return
            if result is False:
                if self._nav_retry < 1:
                    self._nav_retry += 1
                    self._set_state("NAV_TO_PLACE", "retry nav place")
                else:
                    self._set_state("RETURN_TO_NEST", "nav place failed")
            return

        if self._state == "PLACE_RELEASE":
            if self._state_elapsed() < 0.1:
                self._gripper.open_gripper()
            if self._state_elapsed() >= self._place_wait:
                self._set_state("VERIFY_PLACE", "release issued")
            return

        if self._state == "VERIFY_PLACE":
            if self._state_elapsed() < 0.1:
                self._gripper.close_gripper()
            if self._state_elapsed() < self._verify_wait:
                return
            if self._check_place_success() or self._check_pantry_region():
                self._placed_crates.add(self._current_crate_id)
                self._plan_index += 1
                if self._allow_second_pick and self._plan_index >= len(self._plan):
                    if self._elapsed() <= 50.0:
                        self._plan.append("adjacent_pantry")
                    self._allow_second_pick = False
                if self._plan_index < len(self._plan):
                    self._set_state("SELECT_CRATE", "next task")
                else:
                    self._set_state("RETURN_TO_NEST", "plan complete")
            else:
                self._set_state("RETURN_TO_NEST", "place verify failed")
            return

        if self._state == "RECOVERY":
            if self._grasp_retry < 1:
                if self._state_elapsed() < self._backup_duration:
                    self._send_cmd_vel(self._backup_speed)
                    return
                self._send_cmd_vel(0.0)
                self._grasp_retry += 1
                self._set_state("ALIGN_TO_CRATE", "retry grasp after backup")
            else:
                self._failed_crates.add(self._current_crate_id)
                self._set_state("SELECT_CRATE", "grasp retries exhausted")
            return

        if self._state == "RETURN_TO_NEST":
            nest_pose = _make_pose_stamped(self._nest_x, self._nest_y, self._nest_yaw, self._global_frame)
            if self._state_elapsed() < 0.1:
                self._publish_goal(nest_pose, "return")
            result = self._nav_step(nest_pose, self._timeout_return)
            if result is True:
                self._set_state("DONE", "returned to nest")
            elif result is False:
                self._set_state("DONE", "return failed")
            return

    def _timeout_for_state(self) -> float:
        return {
            "MOVE_CURSOR": self._timeout_move_cursor,
            "NAV_TO_CURSOR": self._timeout_nav,
            "HOOK_GRAB": self._timeout_hook,
            "NAV_CURSOR_TO_TARGET": self._timeout_nav,
            "HOOK_RELEASE": self._timeout_hook,
            "SELECT_CRATE": self._timeout_select_crate,
            "NAV_TO_PREGRASP": self._timeout_nav,
            "ALIGN_TO_CRATE": self._timeout_align,
            "GRIPPER_CLOSE": self._timeout_gripper,
            "VERIFY_GRASP": self._timeout_verify,
            "NAV_TO_PLACE": self._timeout_nav,
            "PLACE_RELEASE": self._timeout_gripper,
            "VERIFY_PLACE": self._timeout_verify,
            "RECOVERY": self._timeout_align,
        }.get(self._state, self._timeout_nav)


def main() -> None:
    rclpy.init()
    node = TaskManagerFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
