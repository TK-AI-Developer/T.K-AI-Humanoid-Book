#!/usr/bin/env python3
"""
Data Retention System for Digital Twin Simulation

This script implements data retention functionality to store simulation data
for 24 hours to enable debugging and validation of AI algorithms.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Header
import time
import json
import os
from datetime import datetime, timedelta
import threading


class DataRetentionSystem(Node):
    def __init__(self):
        super().__init__('data_retention_system')
        
        # Directory to store data
        self.data_dir = '/tmp/digital_twin_data'  # In a real implementation, this would be configurable
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)
        
        # Data storage
        self.sensor_data_buffer = {
            'lidar': [],
            'imu': [],
            'camera': [],
            'model_states': [],
            'commands': []
        }
        
        # Buffer size limits (to prevent excessive memory usage)
        self.buffer_size_limit = 10000  # Store up to 10,000 entries per type
        
        # Data retention period (24 hours in seconds)
        self.retention_period = 24 * 60 * 60  # 24 hours in seconds
        
        # Subscribers for data retention
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
        
        self.model_states_subscription = self.create_subscription(
            ModelStates,
            '/model_states',
            self.model_states_callback,
            10)
        
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Timer for periodic data saving
        self.save_timer = self.create_timer(60.0, self.save_data_to_disk)  # Save every minute
        
        # Timer for data cleanup
        self.cleanup_timer = self.create_timer(300.0, self.cleanup_old_data)  # Cleanup every 5 minutes
        
        self.get_logger().info(f'Data retention system initialized, storing data in: {self.data_dir}')

    def lidar_callback(self, msg):
        """Handle LiDAR data retention"""
        self.store_data('lidar', {
            'timestamp': time.time(),
            'ranges': list(msg.ranges),
            'intensities': list(msg.intensities),
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'time_increment': msg.time_increment,
            'scan_time': msg.scan_time,
            'range_min': msg.range_min,
            'range_max': msg.range_max
        })

    def imu_callback(self, msg):
        """Handle IMU data retention"""
        self.store_data('imu', {
            'timestamp': time.time(),
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            }
        })

    def camera_callback(self, msg):
        """Handle camera data retention"""
        # For efficiency, we only store metadata for images
        # In a real implementation, you might store actual image data
        self.store_data('camera', {
            'timestamp': time.time(),
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding,
            'is_bigendian': msg.is_bigendian,
            'step': msg.step,
            # 'data_size': len(msg.data)  # Uncomment if storing image data
        })

    def model_states_callback(self, msg):
        """Handle model states retention"""
        model_data = {
            'timestamp': time.time(),
            'names': list(msg.name),
            'poses': [],
            'twists': []
        }
        
        for pose in msg.pose:
            model_data['poses'].append({
                'position': {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z},
                'orientation': {'x': pose.orientation.x, 'y': pose.orientation.y, 'z': pose.orientation.z, 'w': pose.orientation.w}
            })
        
        for twist in msg.twist:
            model_data['twists'].append({
                'linear': {'x': twist.linear.x, 'y': twist.linear.y, 'z': twist.linear.z},
                'angular': {'x': twist.angular.x, 'y': twist.angular.y, 'z': twist.angular.z}
            })
        
        self.store_data('model_states', model_data)

    def cmd_vel_callback(self, msg):
        """Handle command velocity retention"""
        self.store_data('commands', {
            'timestamp': time.time(),
            'linear': {'x': msg.linear.x, 'y': msg.linear.y, 'z': msg.linear.z},
            'angular': {'x': msg.angular.x, 'y': msg.angular.y, 'z': msg.angular.z}
        })

    def store_data(self, data_type, data):
        """Store data in the buffer"""
        self.sensor_data_buffer[data_type].append(data)
        
        # Remove oldest entries if buffer is too large
        if len(self.sensor_data_buffer[data_type]) > self.buffer_size_limit:
            # Keep the most recent entries
            self.sensor_data_buffer[data_type] = self.sensor_data_buffer[data_type][-self.buffer_size_limit:]

    def save_data_to_disk(self):
        """Save data buffers to disk"""
        try:
            # Create a timestamped file for each data type
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            
            for data_type, buffer in self.sensor_data_buffer.items():
                if buffer:  # Only save if there's data
                    filename = os.path.join(self.data_dir, f'{data_type}_{timestamp}.json')
                    with open(filename, 'w') as f:
                        json.dump(buffer, f)
                    
                    # Clear the buffer after saving (keeping recent data for immediate access)
                    # In a real implementation, you might want to keep some recent data
                    self.sensor_data_buffer[data_type] = []
                    
                    self.get_logger().debug(f'Saved {len(buffer)} {data_type} entries to {filename}')
        except Exception as e:
            self.get_logger().error(f'Error saving data to disk: {str(e)}')

    def cleanup_old_data(self):
        """Remove data files older than the retention period"""
        try:
            current_time = time.time()
            removed_count = 0
            
            for filename in os.listdir(self.data_dir):
                filepath = os.path.join(self.data_dir, filename)
                
                # Get file modification time
                file_time = os.path.getmtime(filepath)
                
                # Check if file is older than retention period
                if current_time - file_time > self.retention_period:
                    os.remove(filepath)
                    removed_count += 1
                    self.get_logger().debug(f'Removed old data file: {filename}')
            
            if removed_count > 0:
                self.get_logger().info(f'Removed {removed_count} old data files')
        except Exception as e:
            self.get_logger().error(f'Error cleaning up old data: {str(e)}')

    def get_data_since(self, data_type, since_time):
        """Get data of a specific type since a given time"""
        # In a real implementation, this would retrieve data from disk
        # For this example, we'll just return recent data from memory
        recent_data = []
        for entry in self.sensor_data_buffer[data_type]:
            if entry['timestamp'] >= since_time:
                recent_data.append(entry)
        return recent_data

    def export_data_range(self, start_time, end_time, output_dir):
        """Export data for a specific time range to a given directory"""
        # In a real implementation, this would search through stored files
        # and extract data within the specified time range
        self.get_logger().info(f'Exporting data from {start_time} to {end_time} to {output_dir}')
        
        # Create output directory if it doesn't exist
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        # This is a simplified implementation
        # In a real system, you would search through all stored data files
        for data_type in self.sensor_data_buffer.keys():
            # Filter data within time range
            filtered_data = []
            for entry in self.sensor_data_buffer[data_type]:
                if start_time <= entry['timestamp'] <= end_time:
                    filtered_data.append(entry)
            
            # Save to output directory
            if filtered_data:
                filename = os.path.join(output_dir, f'{data_type}_export.json')
                with open(filename, 'w') as f:
                    json.dump(filtered_data, f)
                
                self.get_logger().info(f'Exported {len(filtered_data)} {data_type} entries to {filename}')


def main(args=None):
    rclpy.init(args=args)
    retention_system = DataRetentionSystem()
    
    try:
        rclpy.spin(retention_system)
    except KeyboardInterrupt:
        pass
    finally:
        retention_system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()