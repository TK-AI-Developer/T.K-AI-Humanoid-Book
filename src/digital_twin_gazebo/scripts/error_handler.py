#!/usr/bin/env python3
"""
Error Handling and Recovery System for Digital Twin Simulation

This script implements error handling for sensor data corruption and other
simulation issues in the digital twin system.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from std_msgs.msg import Header, String
from builtin_interfaces.msg import Time
import numpy as np
import math
import time
from collections import deque


class ErrorHandler(Node):
    def __init__(self):
        super().__init__('error_handler')
        
        # Declare parameters
        self.declare_parameter('lidar_error_threshold', 0.1)  # Fraction of invalid readings
        self.declare_parameter('imu_error_threshold', 0.05)  # Fraction of invalid readings
        self.declare_parameter('camera_error_threshold', 0.2)  # Fraction of invalid readings
        self.declare_parameter('history_size', 10)  # Number of previous readings to store
        
        # Get parameters
        self.lidar_error_threshold = self.get_parameter('lidar_error_threshold').value
        self.imu_error_threshold = self.get_parameter('imu_error_threshold').value
        self.camera_error_threshold = self.get_parameter('camera_error_threshold').value
        self.history_size = self.get_parameter('history_size').value
        
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
        
        self.camera_subscription = self.create_subscription(
            Image,
            '/sensor_data/depth_camera',
            self.camera_callback,
            10)
        
        # Publishers for error reports and corrected data
        self.error_publisher = self.create_publisher(String, '/system/errors', 10)
        self.corrected_lidar_publisher = self.create_publisher(LaserScan, '/sensor_data/lidar_corrected', 10)
        self.corrected_imu_publisher = self.create_publisher(Imu, '/sensor_data/imu_corrected', 10)
        self.corrected_camera_publisher = self.create_publisher(Image, '/sensor_data/depth_camera_corrected', 10)
        
        # History buffers for each sensor
        self.lidar_history = deque(maxlen=self.history_size)
        self.imu_history = deque(maxlen=self.history_size)
        self.camera_history = deque(maxlen=self.history_size)
        
        self.get_logger().info('Error Handler initialized')

    def lidar_callback(self, msg):
        """Handle LiDAR data with error detection and correction"""
        # Check for corrupted data
        invalid_count = 0
        for r in msg.ranges:
            if math.isnan(r) or math.isinf(r):
                invalid_count += 1
        
        invalid_fraction = invalid_count / len(msg.ranges) if len(msg.ranges) > 0 else 0
        
        if invalid_fraction > self.lidar_error_threshold:
            self.get_logger().warn(f'LiDAR data corruption detected: {invalid_fraction:.2%} invalid readings')
            
            # Publish error message
            error_msg = String()
            error_msg.data = f'LiDAR corruption: {invalid_fraction:.2%} invalid readings'
            self.error_publisher.publish(error_msg)
            
            # Correct the data using history
            corrected_msg = self.correct_lidar_data(msg)
        else:
            # Valid data, add to history
            self.lidar_history.append(msg)
            corrected_msg = msg
        
        # Publish corrected or original data
        self.corrected_lidar_publisher.publish(corrected_msg)

    def imu_callback(self, msg):
        """Handle IMU data with error detection and correction"""
        # Check for corrupted data
        invalid_values = 0
        total_values = 0
        
        # Check orientation
        orientation_values = [
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ]
        for val in orientation_values:
            if math.isnan(val) or math.isinf(val):
                invalid_values += 1
            total_values += 1
        
        # Check angular velocity
        angular_values = [
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ]
        for val in angular_values:
            if math.isnan(val) or math.isinf(val):
                invalid_values += 1
            total_values += 1
        
        # Check linear acceleration
        linear_values = [
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        ]
        for val in linear_values:
            if math.isnan(val) or math.isinf(val):
                invalid_values += 1
            total_values += 1
        
        invalid_fraction = invalid_values / total_values if total_values > 0 else 0
        
        if invalid_fraction > self.imu_error_threshold:
            self.get_logger().warn(f'IMU data corruption detected: {invalid_fraction:.2%} invalid readings')
            
            # Publish error message
            error_msg = String()
            error_msg.data = f'IMU corruption: {invalid_fraction:.2%} invalid readings'
            self.error_publisher.publish(error_msg)
            
            # Correct the data using history
            corrected_msg = self.correct_imu_data(msg)
        else:
            # Valid data, add to history
            self.imu_history.append(msg)
            corrected_msg = msg
        
        # Publish corrected or original data
        self.corrected_imu_publisher.publish(corrected_msg)

    def camera_callback(self, msg):
        """Handle camera data with error detection and correction"""
        # For camera data, we might check for issues like all zeros, wrong dimensions, etc.
        # This is a simplified check - in reality, you'd want more sophisticated validation
        
        # Check if image data is all zeros (a common corruption)
        # For this example, we'll just check if the step*height equals the data size
        expected_size = msg.step * msg.height
        actual_size = len(msg.data)
        
        if expected_size != actual_size:
            self.get_logger().warn(f'Camera data corruption detected: size mismatch (expected {expected_size}, got {actual_size})')
            
            # Publish error message
            error_msg = String()
            error_msg.data = f'Camera corruption: size mismatch (expected {expected_size}, got {actual_size})'
            self.error_publisher.publish(error_msg)
            
            # Correct the data using history (or return last good data)
            corrected_msg = self.correct_camera_data(msg)
        else:
            # Valid data, add to history
            self.camera_history.append(msg)
            corrected_msg = msg
        
        # Publish corrected or original data
        self.corrected_camera_publisher.publish(corrected_msg)

    def correct_lidar_data(self, original_msg):
        """Correct LiDAR data using historical data"""
        if not self.lidar_history:
            # No history to use for correction, return original with invalid values replaced
            corrected_msg = LaserScan()
            corrected_msg.header = original_msg.header
            corrected_msg.angle_min = original_msg.angle_min
            corrected_msg.angle_max = original_msg.angle_max
            corrected_msg.angle_increment = original_msg.angle_increment
            corrected_msg.time_increment = original_msg.time_increment
            corrected_msg.scan_time = original_msg.scan_time
            corrected_msg.range_min = original_msg.range_min
            corrected_msg.range_max = original_msg.range_max
            
            corrected_msg.ranges = []
            for r in original_msg.ranges:
                if math.isnan(r) or math.isinf(r):
                    corrected_msg.ranges.append(original_msg.range_max)  # Use max range as default
                else:
                    corrected_msg.ranges.append(r)
            
            corrected_msg.intensities = original_msg.intensities  # Copy intensities as-is
            
            return corrected_msg
        else:
            # Use historical data to interpolate missing values
            last_valid_msg = self.lidar_history[-1]
            
            corrected_msg = LaserScan()
            corrected_msg.header = original_msg.header
            corrected_msg.angle_min = original_msg.angle_min
            corrected_msg.angle_max = original_msg.angle_max
            corrected_msg.angle_increment = original_msg.angle_increment
            corrected_msg.time_increment = original_msg.time_increment
            corrected_msg.scan_time = original_msg.scan_time
            corrected_msg.range_min = original_msg.range_min
            corrected_msg.range_max = original_msg.range_max
            
            corrected_msg.ranges = []
            for i, r in enumerate(original_msg.ranges):
                if math.isnan(r) or math.isinf(r):
                    # Use corresponding value from last valid message if available
                    if i < len(last_valid_msg.ranges):
                        corrected_value = last_valid_msg.ranges[i]
                        if math.isnan(corrected_value) or math.isinf(corrected_value):
                            corrected_msg.ranges.append(original_msg.range_max)  # Fallback to max range
                        else:
                            corrected_msg.ranges.append(corrected_value)
                    else:
                        corrected_msg.ranges.append(original_msg.range_max)  # Fallback to max range
                else:
                    corrected_msg.ranges.append(r)
            
            corrected_msg.intensities = original_msg.intensities  # Copy intensities as-is
            
            return corrected_msg

    def correct_imu_data(self, original_msg):
        """Correct IMU data using historical data"""
        if not self.imu_history:
            # No history to use for correction, return original with invalid values replaced
            corrected_msg = Imu()
            corrected_msg.header = original_msg.header
            
            # Handle orientation
            if math.isnan(original_msg.orientation.x) or math.isinf(original_msg.orientation.x):
                corrected_msg.orientation.x = 0.0
            else:
                corrected_msg.orientation.x = original_msg.orientation.x
                
            if math.isnan(original_msg.orientation.y) or math.isinf(original_msg.orientation.y):
                corrected_msg.orientation.y = 0.0
            else:
                corrected_msg.orientation.y = original_msg.orientation.y
                
            if math.isnan(original_msg.orientation.z) or math.isinf(original_msg.orientation.z):
                corrected_msg.orientation.z = 0.0
            else:
                corrected_msg.orientation.z = original_msg.orientation.z
                
            if math.isnan(original_msg.orientation.w) or math.isinf(original_msg.orientation.w):
                corrected_msg.orientation.w = 1.0  # Default for identity quaternion
            else:
                corrected_msg.orientation.w = original_msg.orientation.w
            
            # Normalize quaternion
            norm = math.sqrt(
                corrected_msg.orientation.x**2 +
                corrected_msg.orientation.y**2 +
                corrected_msg.orientation.z**2 +
                corrected_msg.orientation.w**2
            )
            if norm > 0:
                corrected_msg.orientation.x /= norm
                corrected_msg.orientation.y /= norm
                corrected_msg.orientation.z /= norm
                corrected_msg.orientation.w /= norm
            
            # Handle angular velocity
            corrected_msg.angular_velocity.x = original_msg.angular_velocity.x if not (math.isnan(original_msg.angular_velocity.x) or math.isinf(original_msg.angular_velocity.x)) else 0.0
            corrected_msg.angular_velocity.y = original_msg.angular_velocity.y if not (math.isnan(original_msg.angular_velocity.y) or math.isinf(original_msg.angular_velocity.y)) else 0.0
            corrected_msg.angular_velocity.z = original_msg.angular_velocity.z if not (math.isnan(original_msg.angular_velocity.z) or math.isinf(original_msg.angular_velocity.z)) else 0.0
            
            # Handle linear acceleration
            corrected_msg.linear_acceleration.x = original_msg.linear_acceleration.x if not (math.isnan(original_msg.linear_acceleration.x) or math.isinf(original_msg.linear_acceleration.x)) else 0.0
            corrected_msg.linear_acceleration.y = original_msg.linear_acceleration.y if not (math.isnan(original_msg.linear_acceleration.y) or math.isinf(original_msg.linear_acceleration.y)) else 0.0
            corrected_msg.linear_acceleration.z = original_msg.linear_acceleration.z if not (math.isnan(original_msg.linear_acceleration.z) or math.isinf(original_msg.linear_acceleration.z)) else 0.0
            
            # Copy covariance matrices
            corrected_msg.orientation_covariance = original_msg.orientation_covariance
            corrected_msg.angular_velocity_covariance = original_msg.angular_velocity_covariance
            corrected_msg.linear_acceleration_covariance = original_msg.linear_acceleration_covariance
            
            return corrected_msg
        else:
            # Use historical data to interpolate missing values
            last_valid_msg = self.imu_history[-1]
            
            corrected_msg = Imu()
            corrected_msg.header = original_msg.header
            
            # Handle orientation with interpolation from history
            corrected_msg.orientation.x = original_msg.orientation.x if not (math.isnan(original_msg.orientation.x) or math.isinf(original_msg.orientation.x)) else last_valid_msg.orientation.x
            corrected_msg.orientation.y = original_msg.orientation.y if not (math.isnan(original_msg.orientation.y) or math.isinf(original_msg.orientation.y)) else last_valid_msg.orientation.y
            corrected_msg.orientation.z = original_msg.orientation.z if not (math.isnan(original_msg.orientation.z) or math.isinf(original_msg.orientation.z)) else last_valid_msg.orientation.z
            corrected_msg.orientation.w = original_msg.orientation.w if not (math.isnan(original_msg.orientation.w) or math.isinf(original_msg.orientation.w)) else last_valid_msg.orientation.w
            
            # Normalize quaternion
            norm = math.sqrt(
                corrected_msg.orientation.x**2 +
                corrected_msg.orientation.y**2 +
                corrected_msg.orientation.z**2 +
                corrected_msg.orientation.w**2
            )
            if norm > 0:
                corrected_msg.orientation.x /= norm
                corrected_msg.orientation.y /= norm
                corrected_msg.orientation.z /= norm
                corrected_msg.orientation.w /= norm
            
            # Handle angular velocity
            corrected_msg.angular_velocity.x = original_msg.angular_velocity.x if not (math.isnan(original_msg.angular_velocity.x) or math.isinf(original_msg.angular_velocity.x)) else last_valid_msg.angular_velocity.x
            corrected_msg.angular_velocity.y = original_msg.angular_velocity.y if not (math.isnan(original_msg.angular_velocity.y) or math.isinf(original_msg.angular_velocity.y)) else last_valid_msg.angular_velocity.y
            corrected_msg.angular_velocity.z = original_msg.angular_velocity.z if not (math.isnan(original_msg.angular_velocity.z) or math.isinf(original_msg.angular_velocity.z)) else last_valid_msg.angular_velocity.z
            
            # Handle linear acceleration
            corrected_msg.linear_acceleration.x = original_msg.linear_acceleration.x if not (math.isnan(original_msg.linear_acceleration.x) or math.isinf(original_msg.linear_acceleration.x)) else last_valid_msg.linear_acceleration.x
            corrected_msg.linear_acceleration.y = original_msg.linear_acceleration.y if not (math.isnan(original_msg.linear_acceleration.y) or math.isinf(original_msg.linear_acceleration.y)) else last_valid_msg.linear_acceleration.y
            corrected_msg.linear_acceleration.z = original_msg.linear_acceleration.z if not (math.isnan(original_msg.linear_acceleration.z) or math.isinf(original_msg.linear_acceleration.z)) else last_valid_msg.linear_acceleration.z
            
            # Copy covariance matrices
            corrected_msg.orientation_covariance = original_msg.orientation_covariance
            corrected_msg.angular_velocity_covariance = original_msg.angular_velocity_covariance
            corrected_msg.linear_acceleration_covariance = original_msg.linear_acceleration_covariance
            
            return corrected_msg

    def correct_camera_data(self, original_msg):
        """Correct camera data using historical data"""
        if not self.camera_history:
            # No history, return original message
            return original_msg
        else:
            # Use last valid camera message
            last_valid_msg = self.camera_history[-1]
            return last_valid_msg


def main(args=None):
    rclpy.init(args=args)
    error_handler = ErrorHandler()
    
    try:
        rclpy.spin(error_handler)
    except KeyboardInterrupt:
        pass
    finally:
        error_handler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()