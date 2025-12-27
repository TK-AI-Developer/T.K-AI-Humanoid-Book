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

## Setting Up Your First Simulation

1. Launch the basic humanoid world:
   ```bash
   ros2 launch digital_twin_gazebo.launch.py
   ```

2. In a new terminal, launch the Unity visualization:
   ```bash
   # Instructions will vary based on your Unity setup
   # Typically involves running the Unity application and connecting to ROS 2
   ```

3. Verify the simulation is running by checking for sensor data:
   ```bash
   ros2 topic echo /sensor_data/imu
   ```

## Running AI Algorithms

1. Source your ROS 2 workspace:
   ```bash
   source install/setup.bash
   ```

2. Run a sample AI algorithm:
   ```bash
   ros2 run ai_examples sample_controller
   ```

## Troubleshooting

- If Gazebo doesn't launch, ensure your graphics drivers are up to date
- If Unity doesn't connect to ROS 2, verify ROS 2 bridge is properly configured
- For sensor data issues, check topic connections with `ros2 topic list`