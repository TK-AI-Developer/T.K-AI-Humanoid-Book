#!/usr/bin/env python3
"""
Perception Flow Processing Node

This node processes sensor data from the digital twin simulation to create
a perception pipeline for AI validation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from std_msgs.msg import Header
import numpy as np
from scipy.spatial.transform import Rotation as R
import math


class PerceptionFlowProcessor(Node):
    def __init__(self):
        super().__init__('perception_flow_processor')
        
        # Parameters
        self.declare_parameter('sensor_topic_lidar', '/sensor_data/lidar')
        self.declare_parameter('sensor_topic_camera', '/sensor_data/depth_camera')
        self.declare_parameter('sensor_topic_imu', '/sensor_data/imu')
        self.declare_parameter('output_topic_processed', '/perception/processed_data')
        self.declare_parameter('complexity_level', 'medium')  # low, medium, high
        self.declare_parameter('enable_noise_models', True)
        
        # Get parameters
        self.lidar_topic = self.get_parameter('sensor_topic_lidar').value
        self.camera_topic = self.get_parameter('sensor_topic_camera').value
        self.imu_topic = self.get_parameter('sensor_topic_imu').value
        self.complexity_level = self.get_parameter('complexity_level').value
        self.enable_noise_models = self.get_parameter('enable_noise_models').value
        
        # Publishers and subscribers
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self.lidar_callback,
            10)
        
        self.camera_subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.camera_callback,
            10)
        
        self.imu_subscription = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10)
        
        # Processed data publisher
        self.processed_publisher = self.create_publisher(
            # In a real implementation, this would be a custom message type
            # from digital_twin_msgs.msg import ProcessedPerceptionData
            # self.processed_publisher = self.create_publisher(ProcessedPerceptionData, self.output_topic, 10)
        )
        
        # Store latest sensor data
        self.latest_lidar_data = None
        self.latest_camera_data = None
        self.latest_imu_data = None
        
        # Processing timers based on complexity level
        if self.complexity_level == 'low':
            self.processing_timer = self.create_timer(0.2, self.process_perception_data)  # 5 Hz
        elif self.complexity_level == 'medium':
            self.processing_timer = self.create_timer(0.1, self.process_perception_data)  # 10 Hz
        else:  # high
            self.processing_timer = self.create_timer(0.05, self.process_perception_data)  # 20 Hz
            
        self.get_logger().info('Perception Flow Processor initialized')

    def lidar_callback(self, msg):
        """Process incoming LiDAR data"""
        self.latest_lidar_data = msg
        # In a real implementation, we would process the LiDAR data here
        # For example: obstacle detection, mapping, etc.
        self.get_logger().debug(f'Received LiDAR data with {len(msg.ranges)} range values')

    def camera_callback(self, msg):
        """Process incoming camera data"""
        self.latest_camera_data = msg
        # In a real implementation, we would process the camera data here
        # For example: object detection, SLAM, etc.
        self.get_logger().debug(f'Received camera data: {msg.width}x{msg.height}')

    def imu_callback(self, msg):
        """Process incoming IMU data"""
        self.latest_imu_data = msg
        # In a real implementation, we would process the IMU data here
        # For example: orientation estimation, motion tracking, etc.
        self.get_logger().debug(f'Received IMU data: orientation=({msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}, {msg.orientation.w})')

    def process_perception_data(self):
        """Main perception processing function"""
        # Check if we have all required sensor data
        if not all([self.latest_lidar_data, self.latest_camera_data, self.latest_imu_data]):
            return
            
        # Apply noise models if enabled
        if self.enable_noise_models:
            self.apply_noise_models()
        
        # Perform perception tasks based on complexity level
        processed_data = self.perform_perception_tasks()
        
        # Publish processed data
        # In a real implementation:
        # processed_msg = ProcessedPerceptionData()
        # processed_msg.header = Header()
        # processed_msg.header.stamp = self.get_clock().now().to_msg()
        # processed_msg.header.frame_id = 'base_link'
        # processed_msg.data = processed_data
        # self.processed_publisher.publish(processed_msg)
        
        self.get_logger().info('Processed perception data published')

    def apply_noise_models(self):
        """Apply noise models to sensor data based on configuration"""
        # Apply noise to LiDAR data
        if self.latest_lidar_data and self.enable_noise_models:
            # Add Gaussian noise to range measurements
            noise_std = 0.01  # 1cm standard deviation
            noisy_ranges = []
            for r in self.latest_lidar_data.ranges:
                if not (math.isnan(r) or math.isinf(r)):
                    noisy_range = r + np.random.normal(0, noise_std)
                    noisy_ranges.append(max(0.0, noisy_range))  # Ensure non-negative
                else:
                    noisy_ranges.append(r)  # Keep invalid measurements as-is
            self.latest_lidar_data.ranges = noisy_ranges

        # Apply noise to IMU data
        if self.latest_imu_data and self.enable_noise_models:
            # Add noise to orientation (simplified)
            orientation_noise = 0.001  # Small noise in orientation
            self.latest_imu_data.orientation.x += np.random.normal(0, orientation_noise)
            self.latest_imu_data.orientation.y += np.random.normal(0, orientation_noise)
            self.latest_imu_data.orientation.z += np.random.normal(0, orientation_noise)
            self.latest_imu_data.orientation.w += np.random.normal(0, orientation_noise)
            
            # Normalize quaternion
            norm = math.sqrt(
                self.latest_imu_data.orientation.x**2 +
                self.latest_imu_data.orientation.y**2 +
                self.latest_imu_data.orientation.z**2 +
                self.latest_imu_data.orientation.w**2
            )
            if norm > 0:
                self.latest_imu_data.orientation.x /= norm
                self.latest_imu_data.orientation.y /= norm
                self.latest_imu_data.orientation.z /= norm
                self.latest_imu_data.orientation.w /= norm

    def perform_perception_tasks(self):
        """Perform perception tasks based on complexity level"""
        results = {}
        
        if self.complexity_level in ['medium', 'high']:
            # Perform more complex perception tasks
            results['obstacle_map'] = self.generate_obstacle_map()
            results['feature_points'] = self.extract_feature_points()
            results['motion_estimate'] = self.estimate_motion()
        else:
            # Perform basic perception tasks
            results['obstacle_distances'] = self.get_obstacle_distances()
            results['orientation'] = self.get_orientation_from_imu()
        
        return results

    def generate_obstacle_map(self):
        """Generate a simple obstacle map from LiDAR data"""
        if not self.latest_lidar_data:
            return []
        
        # Convert LiDAR ranges to Cartesian coordinates
        angles = [
            self.latest_lidar_data.angle_min + i * self.latest_lidar_data.angle_increment
            for i in range(len(self.latest_lidar_data.ranges))
        ]
        
        obstacles = []
        for i, r in enumerate(self.latest_lidar_data.ranges):
            if not (math.isnan(r) or math.isinf(r)) and r <= self.latest_lidar_data.range_max:
                x = r * math.cos(angles[i])
                y = r * math.sin(angles[i])
                obstacles.append((x, y))
        
        return obstacles

    def extract_feature_points(self):
        """Extract feature points from camera data"""
        # In a real implementation, this would process the image data
        # to extract features like edges, corners, or objects
        # For this example, we'll return a placeholder
        return [(0.5, 0.5), (0.2, 0.8), (0.9, 0.1)]  # Placeholder feature points

    def estimate_motion(self):
        """Estimate motion using sensor fusion"""
        # Combine IMU and LiDAR data to estimate motion
        # This is a simplified implementation
        if not self.latest_imu_data:
            return {'linear': (0, 0, 0), 'angular': (0, 0, 0)}
        
        # Extract angular velocity from IMU
        angular_velocity = (
            self.latest_imu_data.angular_velocity.x,
            self.latest_imu_data.angular_velocity.y,
            self.latest_imu_data.angular_velocity.z
        )
        
        # For linear velocity, we would typically use additional sensors or estimation
        # For now, return a placeholder
        linear_velocity = (0.0, 0.0, 0.0)
        
        return {'linear': linear_velocity, 'angular': angular_velocity}

    def get_obstacle_distances(self):
        """Get minimum distances in different directions from LiDAR"""
        if not self.latest_lidar_data:
            return {}
        
        # Divide the scan into sectors and find minimum distance in each
        num_sectors = 8
        sector_size = len(self.latest_lidar_data.ranges) // num_sectors
        sectors = {}
        
        for i in range(num_sectors):
            start_idx = i * sector_size
            end_idx = start_idx + sector_size if i < num_sectors - 1 else len(self.latest_lidar_data.ranges)
            
            sector_ranges = [
                r for r in self.latest_lidar_data.ranges[start_idx:end_idx] 
                if not (math.isnan(r) or math.isinf(r))
            ]
            
            if sector_ranges:
                sectors[f'sector_{i}'] = min(sector_ranges)
            else:
                sectors[f'sector_{i}'] = float('inf')
        
        return sectors

    def get_orientation_from_imu(self):
        """Get orientation from IMU data"""
        if not self.latest_imu_data:
            return (0, 0, 0)  # Roll, pitch, yaw
        
        # Convert quaternion to Euler angles
        q = self.latest_imu_data.orientation
        r = R.from_quat([q.x, q.y, q.z, q.w])
        euler_angles = r.as_euler('xyz', degrees=True)
        
        return tuple(euler_angles)


def main(args=None):
    rclpy.init(args=args)
    perception_processor = PerceptionFlowProcessor()
    
    try:
        rclpy.spin(perception_processor)
    except KeyboardInterrupt:
        pass
    finally:
        perception_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()