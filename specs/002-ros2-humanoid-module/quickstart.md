# Quickstart Guide: Digital Twin Simulation Environment

## Prerequisites

- ROS 2 Humble Hawksbill (or later)
- Gazebo Garden (or compatible version)
- Unity 2022.3 LTS (or later)
- Python 3.8+ with ROS 2 Python packages
- Git for version control

## Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. Install ROS 2 dependencies:
   ```bash
   cd src
   rosdep install --from-paths . --ignore-src -r -y
   ```

3. Build the ROS 2 packages:
   ```bash
   colcon build
   source install/setup.bash
   ```

## Module 2: Digital Twin Development

The module follows a progressive approach: Physics → Visualization → Perception

### Chapter 1: Gazebo Physics
1. Set up the physics environment:
   ```bash
   ros2 launch digital_twin_gazebo physics_world.launch.py
   ```
2. Verify physics behaviors (gravity, collisions):
   ```bash
   ros2 topic echo /model_states
   ```

### Chapter 2: Unity Rendering
1. Launch the Unity visualization:
   ```bash
   # Instructions will vary based on your Unity setup
   # Typically involves running the Unity application and connecting to ROS 2
   ```
2. Import URDF models and verify rendering:
   ```bash
   # Check that URDF models are properly loaded and rendered
   ```

### Chapter 3: Sensors
1. Enable sensor simulation:
   ```bash
   ros2 launch digital_twin_gazebo sensor_world.launch.py
   ```
2. Verify sensor data streams:
   ```bash
   ros2 topic echo /sensor_data/lidar
   ros2 topic echo /sensor_data/imu
   ros2 topic echo /sensor_data/depth_camera
   ```

## Documentation Output

The completed module will generate 3 Markdown files in `/docs/module-2/`:
- `gazebo-physics.md` - Physics simulation chapter
- `unity-rendering.md` - Visualization chapter
- `sensor-integration.md` - Sensor integration chapter

## Troubleshooting

- If Gazebo doesn't launch, ensure your graphics drivers are up to date
- If Unity doesn't connect to ROS 2, verify ROS 2 bridge is properly configured
- For sensor data issues, check topic connections with `ros2 topic list`