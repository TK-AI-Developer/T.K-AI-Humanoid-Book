#!/usr/bin/env python3
"""
AI Validation Exercise for Digital Twin Simulation

This script implements an AI validation exercise that tests students'
ability to complete AI validation using the digital twin system.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
import numpy as np
import math
import time
from collections import deque


class AIValidationExercise(Node):
    def __init__(self):
        super().__init__('ai_validation_exercise')
        
        # Declare parameters
        self.declare_parameter('target_position_x', 5.0)
        self.declare_parameter('target_position_y', 5.0)
        self.declare_parameter('success_threshold', 0.5)  # meters
        self.declare_parameter('max_time', 300.0)  # seconds (5 minutes)
        
        # Get parameters
        self.target_x = self.get_parameter('target_position_x').value
        self.target_y = self.get_parameter('target_position_y').value
        self.success_threshold = self.get_parameter('success_threshold').value
        self.max_time = self.get_parameter('max_time').value
        
        # Publishers for commands and feedback
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.result_publisher = self.create_publisher(String, '/validation/result', 10)
        
        # Subscribers for sensor data
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/sensor_data/lidar',
            self.lidar_callback,
            10)
        
        self.imu_subscription = self.create_subscription(
            Imu,
            '/sensor_data/imu',
            self.imu_callback,
            10)
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',  # Assuming odometry is published
            self.odom_callback,
            10)
        
        # Exercise state
        self.start_time = time.time()
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.lidar_data = None
        self.imu_data = None
        self.exercise_completed = False
        self.exercise_successful = False
        
        # Timer for exercise management
        self.exercise_timer = self.create_timer(1.0, self.check_exercise_status)
        
        self.get_logger().info(f'AI Validation Exercise started. Navigate to ({self.target_x}, {self.target_y}) within {self.max_time} seconds.')

    def lidar_callback(self, msg):
        """Handle LiDAR data for navigation"""
        self.lidar_data = msg

    def imu_callback(self, msg):
        """Handle IMU data for orientation"""
        self.imu_data = msg
        # Extract orientation from IMU
        q = msg.orientation
        # Convert quaternion to yaw (simplified)
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def odom_callback(self, msg):
        """Handle odometry data for position"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Update orientation from odometry too
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def check_exercise_status(self):
        """Check if the exercise is completed successfully"""
        if self.exercise_completed:
            return
            
        # Calculate distance to target
        dist_to_target = math.sqrt((self.current_x - self.target_x)**2 + (self.current_y - self.target_y)**2)
        
        # Check if we're within the success threshold
        if dist_to_target <= self.success_threshold:
            self.exercise_successful = True
            self.exercise_completed = True
            
            result_msg = String()
            result_msg.data = f'SUCCESS: Reached target position ({self.target_x}, {self.target_y}) within {time.time() - self.start_time:.2f} seconds'
            self.result_publisher.publish(result_msg)
            
            self.get_logger().info(f'Exercise completed successfully in {time.time() - self.start_time:.2f} seconds!')
            
            # Stop the robot
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            
        # Check if time has run out
        elif time.time() - self.start_time > self.max_time:
            self.exercise_completed = True
            
            result_msg = String()
            result_msg.data = f'FAILED: Time limit exceeded. Final position: ({self.current_x:.2f}, {self.current_y:.2f}), Distance to target: {dist_to_target:.2f}m'
            self.result_publisher.publish(result_msg)
            
            self.get_logger().info(f'Exercise failed - time limit exceeded. Final position: ({self.current_x:.2f}, {self.current_y:.2f})')
            
            # Stop the robot
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)

    def simple_navigation_algorithm(self):
        """
        A simple navigation algorithm that students can use as a baseline
        or improve upon for the validation exercise.
        """
        if self.exercise_completed:
            return
            
        # Calculate direction to target
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle difference
        angle_diff = target_angle - self.current_yaw
        # Normalize angle difference to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Create command
        cmd = Twist()
        
        # Set angular velocity proportional to angle error
        cmd.angular.z = max(-1.0, min(1.0, angle_diff * 1.0))  # Proportional control
        
        # Set linear velocity based on distance and angle
        dist_to_target = math.sqrt(dx**2 + dy**2)
        if abs(angle_diff) < 0.2:  # Only move forward if roughly aligned
            cmd.linear.x = max(0.0, min(1.0, dist_to_target * 0.5))  # Proportional control
        else:
            cmd.linear.x = 0.0  # Don't move forward if not aligned
        
        # Publish command
        self.cmd_vel_publisher.publish(cmd)
        
        # Log status
        self.get_logger().debug(f'Position: ({self.current_x:.2f}, {self.current_y:.2f}), Target: ({self.target_x}, {self.target_y}), Distance: {dist_to_target:.2f}m')

    def run_validation_exercise(self):
        """Run the validation exercise"""
        if self.exercise_completed:
            return
            
        # For this example, we'll use the simple navigation algorithm
        # Students would replace this with their own AI algorithm
        self.simple_navigation_algorithm()


def main(args=None):
    rclpy.init(args=args)
    validator = AIValidationExercise()
    
    # Create a timer to run the validation algorithm periodically
    validation_timer = validator.create_timer(0.1, validator.run_validation_exercise)
    
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()