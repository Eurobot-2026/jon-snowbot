#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

# Edit these defaults when you want to change the dummy TF in code.
DEFAULT_X = 1.2
DEFAULT_Y = 0.7
DEFAULT_Z = 0.0
DEFAULT_QX = 0.0
DEFAULT_QY = 0.0
DEFAULT_QZ = 0.0
DEFAULT_QW = 1.0
DEFAULT_FRAME_ID = "map"
DEFAULT_CHILD_FRAME_ID = "base_link"


class DummyTF(Node):
    def __init__(self) -> None:
        super().__init__("dummy_tf")

        self.declare_parameter("x", DEFAULT_X)
        self.declare_parameter("y", DEFAULT_Y)
        self.declare_parameter("z", DEFAULT_Z)
        self.declare_parameter("qx", DEFAULT_QX)
        self.declare_parameter("qy", DEFAULT_QY)
        self.declare_parameter("qz", DEFAULT_QZ)
        self.declare_parameter("qw", DEFAULT_QW)
        self.declare_parameter("frame_id", DEFAULT_FRAME_ID)
        self.declare_parameter("child_frame_id", DEFAULT_CHILD_FRAME_ID)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.get_parameter("frame_id").value
        transform.child_frame_id = self.get_parameter("child_frame_id").value
        transform.transform.translation.x = float(self.get_parameter("x").value)
        transform.transform.translation.y = float(self.get_parameter("y").value)
        transform.transform.translation.z = float(self.get_parameter("z").value)
        transform.transform.rotation.x = float(self.get_parameter("qx").value)
        transform.transform.rotation.y = float(self.get_parameter("qy").value)
        transform.transform.rotation.z = float(self.get_parameter("qz").value)
        transform.transform.rotation.w = float(self.get_parameter("qw").value)

        broadcaster = StaticTransformBroadcaster(self)
        broadcaster.sendTransform(transform)
        self.get_logger().info(
            f"Publishing static TF {transform.header.frame_id} -> "
            f"{transform.child_frame_id} (x={transform.transform.translation.x:.3f}, "
            f"y={transform.transform.translation.y:.3f}, "
            f"z={transform.transform.translation.z:.3f})"
        )


def main() -> None:
    rclpy.init()
    node = DummyTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
