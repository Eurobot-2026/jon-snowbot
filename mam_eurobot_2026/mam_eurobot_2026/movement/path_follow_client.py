#!/usr/bin/env python3
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path as NavPath


class PathFollowClient(Node):
    def __init__(self) -> None:
        super().__init__("path_follow_client")

        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("follow_action", "/follow_path")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "goal_checker")
        self.declare_parameter("progress_checker_id", "progress_checker")
        self.declare_parameter("cancel_on_new_path", True)

        self._path_topic = self.get_parameter("path_topic").value
        self._follow_action = self.get_parameter("follow_action").value
        self._controller_id = self.get_parameter("controller_id").value
        self._goal_checker_id = self.get_parameter("goal_checker_id").value
        self._progress_checker_id = self.get_parameter("progress_checker_id").value
        self._cancel_on_new_path = bool(self.get_parameter("cancel_on_new_path").value)

        self._action_client = ActionClient(self, FollowPath, self._follow_action)
        self.create_subscription(NavPath, self._path_topic, self._on_path, 10)

        self._pending_path: Optional[NavPath] = None
        self._active_goal_handle = None
        self._goal_in_flight = False
        self._cancel_requested = False

        self.get_logger().info(
            f"PathFollowClient listening on {self._path_topic}, sending FollowPath to {self._follow_action}"
        )

    def _on_path(self, msg: NavPath) -> None:
        if not msg.header.frame_id:
            self.get_logger().warn("Received path with empty frame_id; ignoring.")
            return
        if not msg.poses:
            self.get_logger().warn("Received empty path; ignoring.")
            return

        self._pending_path = msg
        if self._goal_in_flight and self._cancel_on_new_path:
            self._request_cancel()
        elif not self._goal_in_flight:
            self._send_pending()

    def _request_cancel(self) -> None:
        if self._active_goal_handle is None:
            self._cancel_requested = True
            return
        cancel_future = self._active_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._on_cancel_done)

    def _on_cancel_done(self, future) -> None:
        _ = future.result()
        self._goal_in_flight = False
        self._active_goal_handle = None
        self._cancel_requested = False
        self._send_pending()

    def _send_pending(self) -> None:
        if self._pending_path is None:
            return
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f"FollowPath action server not available: {self._follow_action}")
            return

        goal = FollowPath.Goal()
        goal.path = self._pending_path
        goal.controller_id = self._controller_id
        goal.goal_checker_id = self._goal_checker_id
        # goal.progress_checker_id = self._progress_checker_id

        self._pending_path = None
        self._goal_in_flight = True

        send_future = self._action_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("FollowPath goal rejected.")
            self._goal_in_flight = False
            self._send_pending()
            return

        self._active_goal_handle = goal_handle
        if self._cancel_requested:
            self._request_cancel()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f"FollowPath failed with status {result.status}")
        self._goal_in_flight = False
        self._active_goal_handle = None
        self._send_pending()


def main() -> None:
    rclpy.init()
    node = PathFollowClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
