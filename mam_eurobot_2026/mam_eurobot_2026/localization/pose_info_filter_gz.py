#!/usr/bin/env python3
import importlib

import rclpy
from rclpy.node import Node


def _load_gz_node_class():
    candidates = (
        "gz.transport15",
        "gz.transport14",
        "gz.transport13",
        "ignition.transport15",
        "ignition.transport14",
        "ignition.transport13",
    )
    for module_name in candidates:
        try:
            module = importlib.import_module(module_name)
            return module.Node
        except ImportError:
            continue
    raise ImportError("Gazebo Transport Python binding is not available.")


def _load_pose_msg_modules():
    candidates = (
        "gz.msgs",
        "ignition.msgs",
        "gz.msgs12",
        "gz.msgs11",
        "gz.msgs10",
    )
    for module_name in candidates:
        try:
            module = importlib.import_module(module_name)
            return module.pose_v_pb2, module.pose_pb2
        except (ImportError, AttributeError):
            continue
    raise ImportError("Gazebo message Python bindings are not available.")


class PoseInfoFilterGz(Node):
    def __init__(self) -> None:
        super().__init__("pose_info_filter_gz")

        self.declare_parameter("world_name", "eurobot_2026_arena")
        self.declare_parameter("target_model_name", "simple_robot")
        self.declare_parameter("out_topic", "/model/simple_robot/pose_gt")
        self.declare_parameter("use_sim_time", False)

        world_name = self.get_parameter("world_name").get_parameter_value().string_value
        target_model_name = self.get_parameter("target_model_name").get_parameter_value().string_value
        out_topic = self.get_parameter("out_topic").get_parameter_value().string_value

        if not out_topic:
            out_topic = f"/model/{target_model_name}/pose_gt"

        in_topic = f"/world/{world_name}/pose/info"

        gz_node_class = _load_gz_node_class()
        pose_v_pb2, pose_pb2 = _load_pose_msg_modules()

        self._gz_node = gz_node_class()
        self._pose_v_pb2 = pose_v_pb2
        self._pose_pb2 = pose_pb2
        self._target_model_name = target_model_name
        self._out_topic = out_topic
        self._published_once = False

        self._publisher = self._gz_node.advertise(self._out_topic, self._pose_pb2.Pose)

        def _on_pose_info(msg, *_args):
            if msg is None:
                return
            for pose in msg.pose:
                if pose.name == self._target_model_name:
                    out_msg = self._pose_pb2.Pose()
                    out_msg.CopyFrom(pose)
                    self._publish_pose(out_msg)
                    if not self._published_once:
                        self._published_once = True
                        self.get_logger().info(
                            f"Publishing {self._target_model_name} pose to {self._out_topic}."
                        )
                    return

        try:
            self._gz_node.subscribe(in_topic, _on_pose_info, self._pose_v_pb2.Pose_V)
        except TypeError:
            self._gz_node.subscribe(in_topic, _on_pose_info)

        self.get_logger().info(
            f"Filtering {in_topic} for model '{self._target_model_name}' -> {self._out_topic}"
        )

    def _publish_pose(self, pose_msg) -> None:
        if self._publisher is not None:
            self._publisher.publish(pose_msg)
            return
        try:
            self._gz_node.publish(self._out_topic, pose_msg)
        except Exception:
            return


def main() -> None:
    rclpy.init()
    try:
        node = PoseInfoFilterGz()
    except ImportError as exc:
        rclpy.shutdown()
        raise SystemExit(str(exc)) from exc
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
