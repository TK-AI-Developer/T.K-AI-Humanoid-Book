# Chapter 1: Gazebo Physics

## Overview

This chapter covers the physics simulation environment for the Digital Twin project. We'll be using Gazebo, a powerful physics simulator that provides realistic simulation of gravity, collisions, and joint constraints. This environment allows AI & Robotics students to test algorithms before deploying to real hardware.

## Key Concepts

### Gravity Simulation
Gravity is a fundamental aspect of our physics simulation. In Gazebo, gravity is represented as a vector that applies constant acceleration to all objects in the simulation. The default gravity setting is (0, 0, -9.81) m/s², representing Earth's gravitational acceleration in the negative Z direction.

### Collision Detection
Gazebo handles collision detection between objects using various algorithms. Our humanoid model includes collision properties for each link, allowing for realistic interaction with the environment and other objects.

### Joint Constraints
Our humanoid model includes several joints with different constraint types:
- Revolute joints for arms and legs (allowing rotation around one axis)
- Fixed joints for permanent connections (like head to torso)

## Implementation

### Setting up the Physics Environment

The physics environment is configured in the `simple_world.sdf` file, where we define:

1. Gravity parameters
2. Physics engine settings (ODE in this case)
3. Simulation step size and real-time update rate

### Configurable Physics Fidelity

We've implemented a configurable physics fidelity system that allows users to adjust between real-time performance and higher accuracy. This system can be configured via the `config/physics.yaml` file, where you can adjust:

- `real_time_factor`: Controls the speed of the simulation relative to real time
- `step_size`: The simulation time step (smaller steps = more accuracy but slower performance)
- `solver_iterations`: Number of iterations for the physics solver (more iterations = more accuracy)

### The Humanoid Model

Our enhanced humanoid model includes:

- A base torso with head
- Two arms with revolute joints
- Two legs with revolute joints
- IMU, LiDAR, and depth camera sensors

The model is defined in URDF (Unified Robot Description Format) and includes both visual and collision properties.

## Testing Physics Behavior

To verify that physics is working correctly:

1. Launch the physics world:
   ```bash
   ros2 launch digital_twin_gazebo physics_world.launch.py
   ```

2. Verify gravity by observing if objects fall naturally:
   ```bash
   ros2 topic echo /model_states
   ```

3. Test collisions by spawning multiple objects and observing their interaction.

## Performance Considerations

For educational purposes, we've implemented a configurable physics fidelity system that allows users to adjust between real-time performance and higher accuracy. This balances learning objectives with hardware constraints.

## Next Steps

Once the physics environment is set up and tested, proceed to Chapter 2: Unity Rendering to set up the visualization component of the Digital Twin.