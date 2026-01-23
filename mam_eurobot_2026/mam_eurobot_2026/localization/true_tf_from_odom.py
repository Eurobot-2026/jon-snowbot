#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class TrueTfFromOdom(Node):
    def __init__(self) -> None:
        super().__init__("true_tf_from_odom")

        self.declare_parameter("model_name", "simple_robot")
        self.declare_parameter("odom_topic", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 5.0)

        model_name = self.get_parameter("model_name").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self._map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self._base_frame = self.get_parameter("base_frame").get_parameter_value().string_value

        if not odom_topic:
            odom_topic = f"/model/{model_name}/odometry"

        self._tf_broadcaster = TransformBroadcaster(self)
        self._first_msg = True
        self._latest_odom = None

        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        publish_period = 1.0 / max(publish_rate, 0.01)
        self._timer = self.create_timer(publish_period, self._publish_latest_tf)

        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.get_logger().info(
            f"Subscribing to {odom_topic} and publishing TF {self._map_frame} -> {self._base_frame}"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _publish_latest_tf(self) -> None:
        if self._latest_odom is None:
            return

        msg = self._latest_odom
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self._map_frame
        tf_msg.child_frame_id = self._base_frame
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation

        self._tf_broadcaster.sendTransform(tf_msg)

        if self._first_msg:
            self._first_msg = False
            self.get_logger().info("Published first map -> base_link TF from odometry.")


def main() -> None:
    rclpy.init()
    node = TrueTfFromOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
