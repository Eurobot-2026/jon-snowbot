#!/usr/bin/env python3
"""
Keyboard teleop for the parallel gripper.
Press 'g' to toggle open/close.
"""

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class GripperKeyboard(Node):
    def __init__(self):
        super().__init__('gripper_keyboard')

        self.declare_parameter(
            'command_topic',
            '/gripper/cmd_pos',
        )
        self.declare_parameter('open_position', 0)
        self.declare_parameter('closed_position', -0.05)

        self.command_topic = self.get_parameter('command_topic').value
        self.open_position = float(self.get_parameter('open_position').value)
        self.closed_position = float(self.get_parameter('closed_position').value)

        self.publisher = self.create_publisher(Float64, self.command_topic, 10)
        self.is_open = True

        self.get_logger().info("Gripper keyboard teleop started.")
        self.get_logger().info("Press 'g' to toggle open/close, 'q' to quit.")
        self.get_logger().info(
            f"Publishing to: {self.command_topic} "
            f"(open={self.open_position}, closed={self.closed_position})"
        )

        self._publish_position(self.open_position)
        self.timer = self.create_timer(0.05, self._poll_keyboard)

    def _publish_position(self, position: float) -> None:
        msg = Float64()
        msg.data = float(position)
        self.publisher.publish(msg)

    def _toggle(self) -> None:
        self.is_open = not self.is_open
        target = self.open_position if self.is_open else self.closed_position
        state = "open" if self.is_open else "closed"
        self._publish_position(target)
        self.get_logger().info(f"Gripper {state}: {target}")

    def _poll_keyboard(self) -> None:
        if not self._key_available():
            return
        ch = sys.stdin.read(1)
        if ch in ('g', 'G'):
            self._toggle()
        elif ch in ('q', 'Q'):
            self.get_logger().info("Quit requested.")
            rclpy.shutdown()

    @staticmethod
    def _key_available() -> bool:
        return bool(select.select([sys.stdin], [], [], 0)[0])


def main(args=None):
    rclpy.init(args=args)

    old_term = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        node = GripperKeyboard()
        rclpy.spin(node)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
