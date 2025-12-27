# ROS 2 API Contracts: Digital Twin Simulation

## Sensor Data Topics

### /sensor_data/lidar
- **Type**: sensor_msgs/LaserScan
- **Description**: LiDAR sensor readings
- **Publishers**: Gazebo simulation
- **Subscribers**: AI algorithms, visualization tools
- **Frequency**: Configurable (typically 10-20 Hz)

### /sensor_data/depth_camera
- **Type**: sensor_msgs/Image
- **Description**: Depth camera image data
- **Publishers**: Gazebo simulation
- **Subscribers**: AI algorithms, visualization tools
- **Frequency**: Configurable (typically 15-30 Hz)

### /sensor_data/imu
- **Type**: sensor_msgs/Imu
- **Description**: IMU sensor readings (orientation, angular velocity, linear acceleration)
- **Publishers**: Gazebo simulation
- **Subscribers**: AI algorithms, state estimation
- **Frequency**: Configurable (typically 50-100 Hz)

## Control Topics

### /cmd_vel
- **Type**: geometry_msgs/Twist
- **Description**: Velocity commands for the robot
- **Publishers**: AI algorithms
- **Subscribers**: Gazebo simulation
- **Frequency**: Variable based on control algorithm

### /joint_commands
- **Type**: std_msgs/Float64MultiArray
- **Description**: Joint position/effort commands
- **Publishers**: AI algorithms
- **Subscribers**: Gazebo simulation
- **Frequency**: Variable based on control algorithm

## State Topics

### /model_states
- **Type**: gazebo_msgs/ModelStates
- **Description**: Current positions and orientations of all models
- **Publishers**: Gazebo simulation
- **Subscribers**: Visualization tools, debugging
- **Frequency**: Configurable (typically 30 Hz)

### /tf and /tf_static
- **Type**: tf2_msgs/TFMessage
- **Description**: Transform tree for coordinate frame relationships
- **Publishers**: Gazebo simulation
- **Subscribers**: All components needing coordinate transforms
- **Frequency**: Variable based on simulation rate