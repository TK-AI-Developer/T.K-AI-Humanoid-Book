# Data Model: Digital Twin Simulation Environment

## Digital Twin Model
- **Name**: Unique identifier for the digital twin
- **Geometry**: 3D model representation (mesh, textures, materials)
- **Physics Properties**: Mass, friction, collision shapes, joint constraints
- **Sensor Configurations**: List of sensors attached to the model
- **State**: Current position, orientation, velocities, joint angles

## Simulation Environment
- **Name**: Unique identifier for the environment
- **Physics Parameters**: Gravity, damping, simulation step size
- **Environmental Elements**: Static objects, terrain, lighting conditions
- **Models**: List of digital twin models present in the environment
- **State**: Current simulation time, running/paused status

## Sensor Data Pipeline
- **Sensor Type**: LiDAR, depth camera, IMU, etc.
- **Configuration**: Parameters specific to each sensor type
- **Output Format**: Data structure of sensor readings
- **Frequency**: Update rate for sensor data
- **Processing**: Transformations applied to raw sensor data
- **Destination**: Where the sensor data is sent (ROS 2 topic, file, etc.)

## State Transitions

### Digital Twin Model States
- **Idle**: Model loaded but simulation not started
- **Initializing**: Simulation parameters being set up
- **Running**: Model is actively being simulated
- **Paused**: Simulation temporarily stopped
- **Error**: Simulation encountered an issue

### Simulation Environment States
- **Created**: Environment initialized
- **Loading**: Models and parameters being loaded
- **Ready**: All components loaded, ready to start simulation
- **Simulating**: Physics simulation is running
- **Stopped**: Simulation has ended