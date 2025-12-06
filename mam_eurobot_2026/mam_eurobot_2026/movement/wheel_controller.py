#!/usr/bin/env python3
"""
ROS 2 Mecanum Drive Controller (Twist Command)
Controls the robot's overall motion (linear X, Y, and angular Z)
by publishing to the '/cmd_vel' topic.
The Gazebo MecanumDrive plugin handles the individual wheel kinematics.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # We now use the standard Twist message
import time

# --- Topic definition based on your launch file and model name ---
# Assuming your robot model name is 'simple_robot'
CMD_VEL_TOPIC = '/model/simple_robot/cmd_vel'


class MecanumController(Node):
    def __init__(self):
        super().__init__('mecanum_controller')

        # Create a single publisher for the command velocity topic
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self.get_logger().info('Mecanum controller initialized.')
        self.get_logger().info(f'Publishing to: {CMD_VEL_TOPIC} (geometry_msgs/msg/Twist)')

    def publish_twist(self, linear_x: float, linear_y: float, angular_z: float):
        """
        Publish a Twist message to control the robot's overall velocity.

        Args:
            linear_x: Linear velocity in the X-direction (forward/backward, m/s).
            linear_y: Linear velocity in the Y-direction (strafe left/right, m/s).
            angular_z: Angular velocity about the Z-axis (turning, rad/s).
        """
        twist_msg = Twist()
        twist_msg.linear.x = float(linear_x)
        twist_msg.linear.y = float(linear_y)
        twist_msg.angular.z = float(angular_z)

        self.cmd_vel_pub.publish(twist_msg)

        self.get_logger().info(
            f'CmdVel - Lin X: {linear_x:.2f}, Lin Y: {linear_y:.2f}, Ang Z: {angular_z:.2f}'
        )

    def stop(self):
        """Stop all motion."""
        self.publish_twist(0.0, 0.0, 0.0)

    def move_forward(self, speed: float = 0.5):
        """Move forward (positive X linear velocity)."""
        self.publish_twist(speed, 0.0, 0.0)

    def strafe_left(self, speed: float = 0.5):
        """Strafe left (positive Y linear velocity)."""
        self.publish_twist(0.0, speed, 0.0)
    
    def rotate_in_place_ccw(self, speed: float = 0.5):
        """Rotate counter-clockwise (positive Z angular velocity)."""
        self.publish_twist(0.0, 0.0, speed)
    
    # You can add more combined movements here, e.g., diagonal
    def move_diagonal_forward_right(self, linear_speed: float = 0.5, angular_speed: float = 0.0):
        """Move diagonally forward and right."""
        self.publish_twist(linear_speed, -linear_speed, angular_speed)


def main(args=None):
    rclpy.init(args=args)
    controller = MecanumController()

    try:
        controller.get_logger().info('Starting Mecanum control demo...')
        
        # Demo sequence uses m/s and rad/s
        speed = 0.5  # m/s or rad/s
        
        # controller.get_logger().info('\n--- Moving Forward (3s) ---')
        # controller.move_forward(speed/2)
        # time.sleep(3)

        # controller.get_logger().info('\n--- Moving Forward (3s) ---')
        # controller.move_forward(-speed/2)
        # time.sleep(3)
        
        # controller.get_logger().info('\n--- Strafing Left (3s) ---')
        # controller.strafe_left(speed/2)
        # time.sleep(3)

        # controller.get_logger().info('\n--- Strafing Left (3s) ---')
        # controller.strafe_left(-speed/2)
        # time.sleep(3)
        
        # controller.get_logger().info('\n--- Rotating CCW (3s) ---')
        # controller.rotate_in_place_ccw(speed * 2) # Use higher angular speed
        # time.sleep(3)
        
        controller.get_logger().info('\n--- Moving Diagonally Forward/Right (3s) ---')
        controller.move_diagonal_forward_right(speed)
        time.sleep(3)
        
        controller.get_logger().info('\n--- Stopping ---')
        controller.stop()
        time.sleep(3)
        
        controller.get_logger().info('Demo complete!')

    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted by user. Stopping robot.')
    finally:
        controller.stop()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()