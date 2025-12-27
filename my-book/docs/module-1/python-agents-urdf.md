---
sidebar_label: 'Python Agents, rclpy, and Humanoid Structure (URDF)'
---

# Chapter 3: Python Agents, rclpy, and Humanoid Structure (URDF)

## Learning Objectives
- Understand the role of rclpy in Python-based AI control
- Learn the separation of decision logic and motor execution
- Understand URDF as a structural model of humanoids
- Learn about links, joints, and kinematic hierarchy

## Introduction

In this chapter, we'll explore how AI agents written in Python can interface with ROS 2 controllers and how the physical structure of humanoid robots is represented using the Unified Robot Description Format (URDF). This connection between AI decision-making and physical robot actuation is essential for creating intelligent embodied systems.

## Core Concepts

### The Role of rclpy in Python-Based AI Control

rclpy is the Python client library for ROS 2, providing the interface between Python programs and the ROS 2 middleware. It allows Python-based AI agents to:

- Create and manage ROS 2 nodes
- Publish and subscribe to topics
- Make service calls and provide services
- Use actions for complex, multi-step processes
- Access ROS 2 parameters and logging systems

<div class="edu-concept">
  <strong>Key Concept:</strong> rclpy serves as the bridge between Python AI agents and the ROS 2 ecosystem, allowing Python programs to participate in the distributed robot communication system.
</div>

### Separation of Decision Logic and Motor Execution

A key design principle in ROS 2 systems is the separation of decision-making from motor execution:

- **Decision Logic**: AI agents that determine what actions to take based on sensor data and goals
- **Motor Execution**: Controller nodes that execute specific motor commands
- **Communication**: Topics and services that connect decision and execution layers

This separation provides several benefits:
- Modularity: Changes in decision logic don't require changes to motor control
- Safety: Motor execution nodes can implement safety checks independent of decision logic
- Flexibility: Different decision-making approaches can use the same motor control infrastructure

### Python AI Agent Connecting to ROS Controllers

Python AI agents can connect to ROS controllers in several ways:

1. **Direct Connection**: The AI agent directly controls robot actions through ROS interfaces
2. **Planning Interface**: The AI agent sends high-level goals to a motion planner, which computes detailed trajectories
3. **Behavior Interface**: The AI agent activates different behavioral modules based on context

<div class="edu-example">
  <strong>Example:</strong> A Python-based pathfinding AI might send navigation goals to a ROS navigation stack, which then handles the low-level motor control to execute the path.
</div>

## URDF: Unified Robot Description Format

URDF (Unified Robot Description Format) is an XML-based format that describes robot kinematic and visual properties. It's essential for:

- Defining the physical structure of robots
- Specifying joint limits and kinematic properties
- Providing visual and collision models for simulation
- Enabling kinematic calculations like inverse kinematics

### Links, Joints, and Kinematic Hierarchy

URDF models consist of two primary elements:

- **Links**: Rigid components of the robot (e.g., arms, legs, torso)
- **Joints**: Connections between links that allow relative motion

The kinematic hierarchy defines how these elements connect:

```
base_link
├── torso
│   ├── head
│   ├── left_arm
│   │   ├── left_forearm
│   │   └── left_hand
│   ├── right_arm
│   │   ├── right_forearm
│   │   └── right_hand
│   ├── left_leg
│   │   └── left_foot
│   └── right_leg
│       └── right_foot
```

<div class="edu-concept">
  <strong>Key Concept:</strong> The kinematic hierarchy in URDF represents the physical connections in a humanoid robot, enabling calculations of how movements in one part affect other parts.
</div>

## Biological Analogies

### Human Body Structure Analogy

URDF's structure mirrors the human body:

- **Links** are like bones - rigid structural elements
- **Joints** are like joints in the human body - allowing specific types of movement
- **Kinematic hierarchy** is like the skeletal system - connecting parts in a logical structure

## Practical Examples

### Example: Controlling a Humanoid Robot's Arm

A Python AI agent might control a humanoid robot's arm using this pattern:

1. **Perception**: Receive sensor data through topics
2. **Decision**: Determine desired end-effector position
3. **Planning**: Use inverse kinematics to compute joint angles
4. **Execution**: Send joint commands to controller nodes

### Code Example with rclpy

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.publisher = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
    def move_arm_to_position(self, joint_positions):
        trajectory = JointTrajectory()
        trajectory.joint_names = ['shoulder_joint', 'elbow_joint', 'wrist_joint']
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 2  # Move to position in 2 seconds
        
        trajectory.points.append(point)
        self.publisher.publish(trajectory)
```

## Diagrams and Visuals

<div class="diagram-container">
  <img src="/img/python-ros-connection.png" alt="Python AI agent connecting to ROS controllers" />
  <div class="diagram-caption">Figure 3.1: Python AI agent connecting to ROS controllers via rclpy</div>
</div>

<div class="diagram-container">
  <img src="/img/urdf-structure.png" alt="URDF structure with links and joints" />
  <div class="diagram-caption">Figure 3.2: URDF structure showing links and joints in a humanoid robot</div>
</div>

## Summary

This chapter has covered the essential connection between Python-based AI agents and physical robot actuation. Through rclpy, Python programs can participate in the ROS 2 communication system, and through URDF, the physical structure of humanoid robots is defined. Understanding these connections is crucial for developing intelligent robotic systems.

## Exercises/Questions

1. What are the advantages of separating decision logic from motor execution in robotic systems?
2. How does URDF enable inverse kinematics calculations for humanoid robots?
3. Describe how a Python AI agent might coordinate with multiple ROS controllers to perform a complex task.

## Next Steps

You've completed Module 1: ROS 2 as the Nervous System of Humanoid Robots. This module established ROS 2 as the foundational middleware that connects AI decision-making to physical humanoid bodies. Continue with Module 2 to learn about Digital Twins and simulation.