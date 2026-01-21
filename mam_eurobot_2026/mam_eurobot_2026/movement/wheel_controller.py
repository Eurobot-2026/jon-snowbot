#!/usr/bin/env python3
"""
ROS 2 Wheel Controller for omni_like_robot
Controls the four wheels of the robot via velocity commands.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time


class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')
        
        # Create publishers for each wheel
        self.front_left_pub = self.create_publisher(Float64, '/omni/front_left_speed', 10)
        self.front_right_pub = self.create_publisher(Float64, '/omni/front_right_speed', 10)
        self.rear_left_pub = self.create_publisher(Float64, '/omni/rear_left_speed', 10)
        self.rear_right_pub = self.create_publisher(Float64, '/omni/rear_right_speed', 10)
        
        self.get_logger().info('Wheel controller initialized. Topics:')
        self.get_logger().info('  - /omni/front_left_speed')
        self.get_logger().info('  - /omni/front_right_speed')
        self.get_logger().info('  - /omni/rear_left_speed')
        self.get_logger().info('  - /omni/rear_right_speed')
    
    def publish_wheel_velocities(self, fl: float, fr: float, rl: float, rr: float):
        """
        Publish velocity commands to all wheels.
        
        Args:
            fl: Front left wheel velocity (rad/s)
            fr: Front right wheel velocity (rad/s)
            rl: Rear left wheel velocity (rad/s)
            rr: Rear right wheel velocity (rad/s)
        """
        msg_fl = Float64(data=fl)
        msg_fr = Float64(data=fr)
        msg_rl = Float64(data=rl)
        msg_rr = Float64(data=rr)
        
        self.front_left_pub.publish(msg_fl)
        self.front_right_pub.publish(msg_fr)
        self.rear_left_pub.publish(msg_rl)
        self.rear_right_pub.publish(msg_rr)
        
        self.get_logger().info(
            f'Velocities - FL: {fl:.2f}, FR: {fr:.2f}, RL: {rl:.2f}, RR: {rr:.2f} rad/s'
        )
    
    def stop(self):
        """Stop all wheels."""
        self.publish_wheel_velocities(0.0, 0.0, 0.0, 0.0)
    
    def move_forward(self, speed: float = 1.0):
        """Move forward with all wheels at same speed."""
        self.publish_wheel_velocities(speed, speed, speed, speed)
    
    def move_backward(self, speed: float = 1.0):
        """Move backward with all wheels at same speed."""
        self.publish_wheel_velocities(-speed, -speed, -speed, -speed)
    
    def turn_left(self, speed: float = 1.0):
        """Turn left (counter-clockwise)."""
        # Left side wheels slower, right side wheels faster
        self.publish_wheel_velocities(speed * 0.5, speed, speed * 0.5, speed)
    
    def turn_right(self, speed: float = 1.0):
        """Turn right (clockwise)."""
        # Right side wheels slower, left side wheels faster
        self.publish_wheel_velocities(speed, speed * 0.5, speed, speed * 0.5)
    
    def rotate_in_place(self, speed: float = 1.0, clockwise: bool = True):
        """Rotate in place."""
        if clockwise:
            # Right side forward, left side backward
            self.publish_wheel_velocities(-speed, speed, -speed, speed)
        else:
            # Left side forward, right side backward
            self.publish_wheel_velocities(speed, -speed, speed, -speed)
    
    def strafe_left(self, speed: float = 1.0):
        """Strafe left (move sideways left)."""
        # All wheels rotate in same direction for omnidirectional left strafe
        self.publish_wheel_velocities(-speed, speed, speed, -speed)
    
    def strafe_right(self, speed: float = 1.0):
        """Strafe right (move sideways right)."""
        # All wheels rotate in same direction for omnidirectional right strafe
        self.publish_wheel_velocities(speed, -speed, -speed, speed)


def main(args=None):
    rclpy.init(args=args)
    controller = WheelController()
    
    try:
        controller.get_logger().info('Starting wheel control demo...')
        
        # Demo sequence
        speed = 2.0  # rad/s
        
        controller.get_logger().info('\n--- Moving Forward ---')
        controller.move_forward(speed)
        time.sleep(3)
        
        controller.get_logger().info('\n--- Moving Backward ---')
        controller.move_backward(speed)
        time.sleep(3)
        
        controller.get_logger().info('\n--- Turning Left ---')
        controller.turn_left(speed)
        time.sleep(3)
        
        controller.get_logger().info('\n--- Turning Right ---')
        controller.turn_right(speed)
        time.sleep(3)
        
        controller.get_logger().info('\n--- Rotating Counter-Clockwise ---')
        controller.rotate_in_place(speed, clockwise=False)
        time.sleep(3)
        
        controller.get_logger().info('\n--- Rotating Clockwise ---')
        controller.rotate_in_place(speed, clockwise=True)
        time.sleep(3)
        
        controller.get_logger().info('\n--- Strafing Left ---')
        controller.strafe_left(speed)
        time.sleep(3)
        
        controller.get_logger().info('\n--- Strafing Right ---')
        controller.strafe_right(speed)
        time.sleep(3)
        
        controller.get_logger().info('\n--- Stopping ---')
        controller.stop()
        time.sleep(1)
        
        controller.get_logger().info('Demo complete!')
        
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted by user')
        controller.stop()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
