#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64
import math
import time

def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

class GoToGoal(Node):

    def __init__(self):
        super().__init__('go_to_goal_deadreckoning')

        # Wheel publishers
        self.fl = self.create_publisher(Float64, '/omni/front_left_speed', 10)
        self.fr = self.create_publisher(Float64, '/omni/front_right_speed', 10)
        self.rl = self.create_publisher(Float64, '/omni/rear_left_speed', 10)
        self.rr = self.create_publisher(Float64, '/omni/rear_right_speed', 10)

        # Subscriber to /goal
        self.create_subscription(
            Pose2D, '/goal', self.goal_callback, 10)

        # Robot state (dead reckoning)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.goal = None

        # Parameters
        self.rot_speed = 7.5   # rad/s
        self.fwd_speed = 2.0   # rad/s (wheel)
        self.yaw_tol = 0.05    # rad
        self.dist_tol = 0.05   # m
        self.dt = 0.05         # control loop period (s)

        # Timer
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('Go-to-goal controller (dead reckoning) ready')

    def goal_callback(self, msg: Pose2D):
        self.goal = msg
        self.get_logger().info(f'New goal: x={msg.x:.2f}, y={msg.y:.2f}')

    def set_wheels(self, fl, fr, rl, rr):
        self.fl.publish(Float64(data=fl))
        self.fr.publish(Float64(data=fr))
        self.rl.publish(Float64(data=rl))
        self.rr.publish(Float64(data=rr))

    def stop(self):
        self.set_wheels(0.0, 0.0, 0.0, 0.0)

    def control_loop(self):
        if self.goal is None:
            return

        dx = self.goal.x - self.x
        dy = self.goal.y - self.y
        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_error = normalize_angle(target_yaw - self.yaw)

        # --- Phase 1: rotate in place ---
        if abs(yaw_error) > self.yaw_tol:
            direction = 1.0 if yaw_error > 0 else -1.0
            w = direction * self.rot_speed
            self.set_wheels(-w, w, -w, w)

            # Update yaw using dead reckoning
            self.yaw += direction * self.rot_speed * self.dt
            self.yaw = normalize_angle(self.yaw)
            return

        # --- Phase 2: move forward ---
        if dist > self.dist_tol:
            self.set_wheels(self.fwd_speed, self.fwd_speed, self.fwd_speed, self.fwd_speed)

            # Update position using simple forward kinematics
            dx_local = self.fwd_speed * self.dt  # distance traveled
            self.x += dx_local * math.cos(self.yaw)
            self.y += dx_local * math.sin(self.yaw)
            return

        # --- Goal reached ---
        self.stop()
        self.goal = None
        self.get_logger().info(f'Goal reached at x={self.x:.2f}, y={self.y:.2f}')

def main():
    rclpy.init()
    node = GoToGoal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
