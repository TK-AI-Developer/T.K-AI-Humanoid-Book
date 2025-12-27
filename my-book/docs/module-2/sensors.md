# Chapter 3: Sensors

## Overview

This chapter covers the sensor integration component of the Digital Twin project. We'll be implementing realistic simulations of LiDAR, depth cameras, and IMUs that provide perception capabilities for the humanoid robot. This enables students to develop perception-to-action AI pipelines that will work with real hardware.

## Key Concepts

### Sensor Simulation
Each sensor type is simulated with realistic noise models and parameters that closely match real hardware. This allows students to learn with realistic data while understanding the limitations and characteristics of actual sensors.

### Perception Flow
The perception flow processes raw sensor data into meaningful information that can be used by AI algorithms. This includes obstacle detection, feature extraction, and state estimation.

### Data Pipeline
The sensor data pipeline handles the flow of data from simulation to processing, including transport via ROS 2 topics, preprocessing, and formatting for AI algorithms.

## Implementation

### Sensor Configuration

Sensors are configured via the `config/perception.yaml` file, where you can adjust:

- **LiDAR**: Frequency, noise models, range, and angular resolution
- **Depth Camera**: Resolution, field of view, and noise characteristics
- **IMU**: Update rate, noise parameters for acceleration and orientation

### Sensor Simulation in Gazebo

Sensors are implemented directly in the URDF model with Gazebo-specific plugins:

1. **LiDAR**: Implemented as a ray sensor with configurable scan parameters
2. **Depth Camera**: Implemented as a depth sensor with RGB and depth output
3. **IMU**: Implemented as an inertial measurement unit with noise models

### Perception Flow Processing

The perception flow processor (`scripts/perception_flow_processor.py`) performs the following tasks:

1. Subscribes to sensor data topics
2. Applies noise models based on configuration
3. Performs perception tasks based on complexity level
4. Publishes processed perception data

The complexity level can be set to "low", "medium", or "high" to adjust the computational requirements and sophistication of the perception algorithms.

### ROS 2 Topics

Sensor data is published to the following ROS 2 topics:

- `/sensor_data/lidar` - LiDAR scan data (sensor_msgs/LaserScan)
- `/sensor_data/depth_camera` - Depth camera image data (sensor_msgs/Image)
- `/sensor_data/imu` - IMU data (sensor_msgs/Imu)

Processed perception data is published to:

- `/perception/processed_data` - Processed perception results

## Testing Sensor Integration

To verify that sensor integration is working correctly:

1. Launch the sensor world:
   ```bash
   ros2 launch digital_twin_gazebo sensor_world.launch.py
   ```

2. Verify LiDAR sensor data:
   ```bash
   ros2 topic echo /sensor_data/lidar
   ```

3. Verify IMU sensor data:
   ```bash
   ros2 topic echo /sensor_data/imu
   ```

4. Verify depth camera data:
   ```bash
   ros2 topic echo /sensor_data/depth_camera
   ```

5. Run the perception flow processor:
   ```bash
   ros2 run digital_twin_gazebo perception_flow_processor.py
   ```

## Performance Considerations

For educational purposes, we've implemented adjustable sensor complexity that allows users to balance between realistic simulation and computational requirements. This is particularly important for institutions with limited hardware resources.

## Next Steps

With all three components of the Digital Twin implemented (Physics, Visualization, and Perception), the system is now complete and ready for integration testing and AI validation exercises.