#!/usr/bin/env python3
"""
Performance Optimization Manager for Digital Twin Simulation

This script implements performance optimization techniques for the digital twin system,
including dynamic adjustment of simulation fidelity based on system resources.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan, Imu, Image
from std_msgs.msg import Header
import psutil
import time
import subprocess
import threading


class PerformanceOptimizer(Node):
    def __init__(self):
        super().__init__('performance_optimizer')
        
        # Declare parameters
        self.declare_parameter('cpu_threshold_high', 80.0)  # Percent
        self.declare_parameter('cpu_threshold_low', 50.0)   # Percent
        self.declare_parameter('memory_threshold_high', 80.0)  # Percent
        self.declare_parameter('memory_threshold_low', 50.0)   # Percent
        self.declare_parameter('adjustment_frequency', 5.0)  # Seconds
        self.declare_parameter('performance_mode', 'balanced')  # low, balanced, high
        
        # Get parameters
        self.cpu_threshold_high = self.get_parameter('cpu_threshold_high').value
        self.cpu_threshold_low = self.get_parameter('cpu_threshold_low').value
        self.memory_threshold_high = self.get_parameter('memory_threshold_high').value
        self.memory_threshold_low = self.get_parameter('memory_threshold_low').value
        self.adjustment_frequency = self.get_parameter('adjustment_frequency').value
        self.performance_mode = self.get_parameter('performance_mode').value
        
        # Current performance settings
        self.current_fidelity = 1.0  # 0.0 to 1.0, where 1.0 is highest fidelity
        self.current_update_rate = 1000.0  # Hz
        self.current_sync_rate = 30.0  # Unity sync rate in Hz
        self.current_sensor_freq = {'lidar': 10.0, 'camera': 15.0, 'imu': 100.0}
        
        # Publishers for performance metrics
        self.cpu_usage_publisher = self.create_publisher(Float32, '/performance/cpu_usage', 10)
        self.memory_usage_publisher = self.create_publisher(Float32, '/performance/memory_usage', 10)
        self.fidelity_publisher = self.create_publisher(Float32, '/performance/current_fidelity', 10)
        
        # Timer for performance monitoring
        self.monitor_timer = self.create_timer(self.adjustment_frequency, self.monitor_performance)
        
        # Initialize with current mode
        self.set_performance_mode(self.performance_mode)
        
        self.get_logger().info('Performance Optimizer initialized')

    def monitor_performance(self):
        """Monitor system resources and adjust performance settings"""
        # Get current CPU and memory usage
        cpu_percent = psutil.cpu_percent(interval=1)
        memory_percent = psutil.virtual_memory().percent
        
        # Publish performance metrics
        cpu_msg = Float32()
        cpu_msg.data = float(cpu_percent)
        self.cpu_usage_publisher.publish(cpu_msg)
        
        memory_msg = Float32()
        memory_msg.data = float(memory_percent)
        self.memory_usage_publisher.publish(memory_msg)
        
        # Adjust settings based on resource usage
        self.adjust_settings(cpu_percent, memory_percent)
        
        # Log current settings
        self.get_logger().debug(
            f'CPU: {cpu_percent:.1f}%, Memory: {memory_percent:.1f}%, '
            f'Fidelity: {self.current_fidelity:.2f}, '
            f'Update Rate: {self.current_update_rate:.1f}Hz'
        )

    def adjust_settings(self, cpu_percent, memory_percent):
        """Adjust simulation settings based on resource usage"""
        # Determine if we need to reduce or increase fidelity
        resource_pressure = max(cpu_percent, memory_percent)
        
        if resource_pressure > self.cpu_threshold_high or resource_pressure > self.memory_threshold_high:
            # High resource pressure - reduce fidelity
            self.reduce_fidelity()
        elif cpu_percent < self.cpu_threshold_low and memory_percent < self.memory_threshold_low:
            # Low resource pressure - can increase fidelity
            self.increase_fidelity()
        # Otherwise, maintain current settings

    def reduce_fidelity(self):
        """Reduce simulation fidelity to improve performance"""
        if self.current_fidelity > 0.2:  # Don't go below 20% fidelity
            self.current_fidelity -= 0.1
            self.current_update_rate *= 0.9  # Reduce update rate by 10%
            
            # Reduce sensor frequencies
            for sensor in self.current_sensor_freq:
                self.current_sensor_freq[sensor] *= 0.9
            
            # Reduce Unity sync rate
            self.current_sync_rate *= 0.9
            
            self.publish_fidelity_update()
            self.get_logger().info(f'Reduced fidelity to {self.current_fidelity:.2f}')

    def increase_fidelity(self):
        """Increase simulation fidelity if resources allow"""
        if self.current_fidelity < 1.0:  # Don't exceed 100% fidelity
            self.current_fidelity += 0.05  # Increase slowly to avoid oscillation
            self.current_update_rate = min(self.current_update_rate * 1.05, 1000.0)  # Cap at 1000Hz
            
            # Increase sensor frequencies (with caps)
            for sensor in self.current_sensor_freq:
                max_freq = {'lidar': 20.0, 'camera': 30.0, 'imu': 200.0}
                self.current_sensor_freq[sensor] = min(self.current_sensor_freq[sensor] * 1.05, max_freq[sensor])
            
            # Increase Unity sync rate (with cap)
            self.current_sync_rate = min(self.current_sync_rate * 1.05, 60.0)  # Cap at 60Hz
            
            self.publish_fidelity_update()
            self.get_logger().info(f'Increased fidelity to {self.current_fidelity:.2f}')

    def set_performance_mode(self, mode):
        """Set performance mode (low, balanced, high)"""
        if mode == 'low':
            self.current_fidelity = 0.4
            self.current_update_rate = 300.0
            self.current_sensor_freq = {'lidar': 5.0, 'camera': 7.0, 'imu': 50.0}
            self.current_sync_rate = 15.0
        elif mode == 'high':
            self.current_fidelity = 1.0
            self.current_update_rate = 1000.0
            self.current_sensor_freq = {'lidar': 20.0, 'camera': 30.0, 'imu': 200.0}
            self.current_sync_rate = 60.0
        else:  # balanced
            self.current_fidelity = 0.7
            self.current_update_rate = 600.0
            self.current_sensor_freq = {'lidar': 10.0, 'camera': 15.0, 'imu': 100.0}
            self.current_sync_rate = 30.0
        
        self.performance_mode = mode
        self.publish_fidelity_update()
        self.get_logger().info(f'Set performance mode to {mode}, fidelity: {self.current_fidelity:.2f}')

    def publish_fidelity_update(self):
        """Publish current fidelity level"""
        fidelity_msg = Float32()
        fidelity_msg.data = self.current_fidelity
        self.fidelity_publisher.publish(fidelity_msg)
        
        # In a real implementation, we would send these settings to Gazebo and Unity
        # For now, we'll just log the intended changes
        self.get_logger().debug(
            f'Fidelity settings - Update rate: {self.current_update_rate:.1f}Hz, '
            f'Sensor freq: {self.current_sensor_freq}, '
            f'Unity sync: {self.current_sync_rate:.1f}Hz'
        )

    def get_system_info(self):
        """Get detailed system information"""
        info = {
            'cpu_count': psutil.cpu_count(),
            'cpu_freq': psutil.cpu_freq(),
            'memory_total': psutil.virtual_memory().total,
            'disk_usage': psutil.disk_usage('/'),
            'system': f'{psutil.LINUX if psutil.LINUX else psutil.WINDOWS if psutil.WINDOWS else "Other"}'
        }
        return info


def main(args=None):
    rclpy.init(args=args)
    optimizer = PerformanceOptimizer()
    
    try:
        rclpy.spin(optimizer)
    except KeyboardInterrupt:
        pass
    finally:
        optimizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()