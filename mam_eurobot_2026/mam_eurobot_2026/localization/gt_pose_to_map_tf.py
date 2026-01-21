#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class GtPoseToMapTf(Node):
    def __init__(self) -> None:
        super().__init__("gt_pose_to_map_tf")

        self.declare_parameter("pose_topic", "/model/simple_robot/pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 2.0)
        # self.declare_parameter("use_sim_time", true)
        self._pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        self._map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self._base_frame = self.get_parameter("base_frame").get_parameter_value().string_value

        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        publish_period = 1.0 / max(publish_rate, 0.01)

        self._tf_broadcaster = TransformBroadcaster(self)
        self._latest_pose = None
        self._first_msg = True

        self.create_subscription(Pose, self._pose_topic, self._pose_cb, 10)
        self.create_timer(publish_period, self._publish_latest_tf)

        self.get_logger().info(
            f"Subscribing to {self._pose_topic} and publishing TF {self._map_frame} -> {self._base_frame}"
        )

    def _pose_cb(self, msg: Pose) -> None:
        self._latest_pose = msg

    def _publish_latest_tf(self) -> None:
        if self._latest_pose is None:
            return

        msg = self._latest_pose
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self._map_frame
        tf_msg.child_frame_id = self._base_frame
        tf_msg.transform.translation.x = msg.position.x
        tf_msg.transform.translation.y = msg.position.y
        tf_msg.transform.translation.z = msg.position.z
        tf_msg.transform.rotation = msg.orientation

        self._tf_broadcaster.sendTransform(tf_msg)

        if self._first_msg:
            self._first_msg = False
            self.get_logger().info("Published first map -> base_link TF from pose topic.")


def main() -> None:
    rclpy.init()
    node = GtPoseToMapTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
