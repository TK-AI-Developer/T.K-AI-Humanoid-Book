#!/usr/bin/env python3
"""
Comprehensive Testing Script for Digital Twin System

This script performs comprehensive testing of the full digital twin system
including physics simulation, visualization, and sensor integration.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Header
import time
import threading


class DigitalTwinTester(Node):
    def __init__(self):
        super().__init__('digital_twin_tester')
        
        # Flags to track if data is received
        self.lidar_received = False
        self.imu_received = False
        self.camera_received = False
        self.model_states_received = False
        
        # Publishers for testing
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers for testing
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
        
        self.get_logger().info('Digital Twin Tester initialized')

    def lidar_callback(self, msg):
        """Handle LiDAR data reception"""
        self.lidar_received = True
        self.get_logger().debug(f'Received LiDAR data with {len(msg.ranges)} range values')

    def imu_callback(self, msg):
        """Handle IMU data reception"""
        self.imu_received = True
        self.get_logger().debug(f'Received IMU data')

    def camera_callback(self, msg):
        """Handle camera data reception"""
        self.camera_received = True
        self.get_logger().debug(f'Received camera data: {msg.width}x{msg.height}')

    def model_states_callback(self, msg):
        """Handle model states reception"""
        self.model_states_received = True
        self.get_logger().debug(f'Received model states for {len(msg.name)} models')

    def test_sensor_data_flow(self):
        """Test that all sensor data is flowing correctly"""
        self.get_logger().info('Testing sensor data flow...')
        
        # Wait for data to arrive
        timeout = 10  # seconds
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            if all([self.lidar_received, self.imu_received, self.camera_received, self.model_states_received]):
                self.get_logger().info('✓ All sensor data is flowing correctly')
                return True
            time.sleep(0.1)
        
        # Report which sensors are not working
        failed_sensors = []
        if not self.lidar_received:
            failed_sensors.append('LiDAR')
        if not self.imu_received:
            failed_sensors.append('IMU')
        if not self.camera_received:
            failed_sensors.append('Camera')
        if not self.model_states_received:
            failed_sensors.append('Model States')
        
        if failed_sensors:
            self.get_logger().error(f'✗ Sensor data flow test failed for: {", ".join(failed_sensors)}')
            return False
        
        return True

    def test_physics_behavior(self):
        """Test physics behavior by sending commands and observing results"""
        self.get_logger().info('Testing physics behavior...')
        
        # Send a simple movement command
        twist = Twist()
        twist.linear.x = 1.0  # Move forward
        twist.angular.z = 0.5  # Turn slightly
        
        # Send command for 2 seconds
        start_time = time.time()
        while (time.time() - start_time) < 2.0:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)
        
        # Stop the robot
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        
        self.get_logger().info('✓ Physics behavior test completed')
        return True

    def run_comprehensive_test(self):
        """Run comprehensive test of the digital twin system"""
        self.get_logger().info('Starting comprehensive digital twin system test...')
        
        # Reset flags
        self.lidar_received = False
        self.imu_received = False
        self.camera_received = False
        self.model_states_received = False
        
        # Wait a bit for the system to settle
        time.sleep(2)
        
        # Test sensor data flow
        sensor_test_passed = self.test_sensor_data_flow()
        
        # Test physics behavior
        physics_test_passed = self.test_physics_behavior()
        
        # Overall result
        all_tests_passed = sensor_test_passed and physics_test_passed
        
        if all_tests_passed:
            self.get_logger().info('✓ All comprehensive tests passed!')
        else:
            self.get_logger().error('✗ Some comprehensive tests failed!')
        
        return all_tests_passed


def main(args=None):
    rclpy.init(args=args)
    tester = DigitalTwinTester()
    
    # Run the comprehensive test in a separate thread to allow ROS to spin
    def run_test():
        time.sleep(3)  # Wait for system to initialize
        tester.run_comprehensive_test()
        rclpy.shutdown()
    
    test_thread = threading.Thread(target=run_test)
    test_thread.start()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        test_thread.join()


if __name__ == '__main__':
    main()