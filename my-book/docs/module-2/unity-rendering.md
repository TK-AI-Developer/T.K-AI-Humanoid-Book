# Chapter 2: Unity Rendering

## Overview

This chapter covers the Unity visualization component of the Digital Twin project. We'll be implementing a Unity application that visualizes the physics simulation from Gazebo in real-time. This provides students with an intuitive, high-fidelity representation of the humanoid robot and its environment.

## Key Concepts

### URDF to Unity Conversion
The process of converting URDF (Unified Robot Description Format) models to Unity-compatible formats is essential for visualization. Our implementation includes tools and workflows to properly import and visualize the humanoid model in Unity.

### ROS 2 Integration
Unity communicates with the ROS 2 ecosystem through a bridge that allows it to receive simulation data from Gazebo. This enables real-time visualization of the physics simulation.

### Lighting and Rendering
Proper lighting and rendering settings are crucial for creating realistic visualizations that help students understand the robot's behavior in its environment.

## Implementation

### Unity Project Structure

The Unity visualization project is organized as follows:

- `Assets/Scenes/` - Contains the main Unity scene files
- `Assets/Scripts/` - Contains all C# scripts for ROS 2 communication, model visualization, lighting, and animation
- `Assets/Models/` - Contains 3D models (though these would typically be generated from URDF)
- `Assets/Materials/` - Contains materials for rendering

### Core Scripts

1. **ROS2Communication.cs**: Handles communication with the ROS 2 ecosystem, receiving model states and sensor data.
2. **HumanoidModelVisualizer.cs**: Manages the visualization of the humanoid model, including joint movements and sensor visualizations.
3. **LightingSystem.cs**: Controls the lighting in the Unity scene, including shadows and dynamic lighting effects.
4. **AnimationSystem.cs**: Manages animations for the humanoid model.
5. **GazeboUnitySynchronizer.cs**: Synchronizes the Unity visualization with the Gazebo physics simulation.

### Setting up the Visualization

The visualization is configured via the `config/visualization.yaml` file, where you can adjust:

- `sync_rate`: The rate at which Unity updates from Gazebo (default 30 Hz)
- `visualization_quality`: Quality level for rendering (options: "low", "medium", "high")
- `enable_shadows`: Whether to enable shadow rendering
- `unity_ip_address` and `unity_port`: Network settings for ROS 2 communication

## Testing Visualization

To verify that visualization is working correctly:

1. Start the Unity application with the Digital Twin visualization scene loaded.
2. Launch the Gazebo simulation with the humanoid model:
   ```bash
   ros2 launch digital_twin_gazebo physics_world.launch.py
   ```
3. Verify that the Unity visualization updates in sync with the physics simulation.

## Performance Considerations

For educational purposes, we've implemented configurable visualization quality that allows users to adjust between visual fidelity and performance. This balances learning objectives with hardware constraints.

## Next Steps

Once the visualization is set up and tested, proceed to Chapter 3: Sensor Integration to implement the perception system of the Digital Twin.