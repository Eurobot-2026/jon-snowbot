#!/usr/bin/env python3
import copy
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path as NavPath
from tf2_ros import Buffer, TransformListener, TransformException

# this node will generate the path listening to topic /task_goal

class TaskGoalPathPlanner(Node):
    def __init__(self) -> None:
        super().__init__("task_goal_path_planner")

        self.declare_parameter("goal_topic", "/task_goal")
        self.declare_parameter("output_path_topic", "/planned_path")
        self.declare_parameter("planner_action_name", "/compute_path_to_pose")
        self.declare_parameter("planner_id", "GridBased")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("plan_rate_hz", 1.0)
        self.declare_parameter("tf_timeout_sec", 0.2)

        self._goal_topic = self.get_parameter("goal_topic").value
        self._output_path_topic = self.get_parameter("output_path_topic").value
        self._planner_action_name = self.get_parameter("planner_action_name").value
        self._planner_id = self.get_parameter("planner_id").value
        self._base_frame = self.get_parameter("base_frame").value
        self._global_frame = self.get_parameter("global_frame").value
        self._plan_rate_hz = float(self.get_parameter("plan_rate_hz").value)
        self._tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._action_client = ActionClient(self, ComputePathToPose, self._planner_action_name)
        self._path_pub = self.create_publisher(NavPath, self._output_path_topic, 10)
        self.create_subscription(PoseStamped, self._goal_topic, self._on_goal_pose, 10)

        self._latest_goal: Optional[PoseStamped] = None
        self._goal_in_flight = False

        period = 1.0 / max(self._plan_rate_hz, 0.001)
        self._timer = self.create_timer(period, self._plan_timer_cb)

        self.get_logger().info(
            f"TaskGoalPathPlanner listening on {self._goal_topic}, publishing {self._output_path_topic}"
        )

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        if not msg.header.frame_id:
            self.get_logger().warn("Received goal pose with empty frame_id; ignoring.")
            return
        self._latest_goal = copy.deepcopy(msg)

    def _plan_timer_cb(self) -> None:
        if self._goal_in_flight or self._latest_goal is None:
            return

        current_pose = self._get_current_pose()
        if current_pose is None:
            return

        goal_pose = self._latest_goal
        if goal_pose.header.frame_id != self._global_frame:
            goal_pose = self._transform_goal(goal_pose)
            if goal_pose is None:
                return

        self._request_plan(current_pose, goal_pose)

    def _get_current_pose(self) -> Optional[PoseStamped]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=float(self._tf_timeout_sec)),
            )
        except TransformException as exc:
            self.get_logger().warn(f"TF lookup failed ({self._global_frame} -> {self._base_frame}): {exc}")
            return None

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._global_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _transform_goal(self, goal_pose: PoseStamped) -> Optional[PoseStamped]:
        try:
            return self._tf_buffer.transform(
                goal_pose,
                self._global_frame,
                timeout=Duration(seconds=float(self._tf_timeout_sec)),
            )
        except TransformException as exc:
            self.get_logger().warn(f"Goal TF failed ({goal_pose.header.frame_id} -> {self._global_frame}): {exc}")
            return None

    def _request_plan(self, start_pose: PoseStamped, goal_pose: PoseStamped) -> None:
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f"Planner action server not available: {self._planner_action_name}")
            return

        goal = ComputePathToPose.Goal()
        goal.start = start_pose
        goal.goal = goal_pose
        goal.use_start = True
        goal.planner_id = self._planner_id

        self._goal_in_flight = True
        send_future = self._action_client.send_goal_async(goal)
        send_future.add_done_callback(self._handle_goal_response)

    def _handle_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Planner rejected path request")
            self._goal_in_flight = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._handle_result)

    def _handle_result(self, future) -> None:
        self._goal_in_flight = False
        result = future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f"Planner failed with status {result.status}")
            return

        path = result.result.path
        if not path.poses:
            self.get_logger().warn("Planner returned an empty path")
            return

        self._path_pub.publish(path)


def main() -> None:
    rclpy.init()
    node = TaskGoalPathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
