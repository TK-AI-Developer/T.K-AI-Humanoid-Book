---
sidebar_label: 'Nodes, Topics, and Services'
---

# Chapter 2: Nodes, Topics, and Services

## Learning Objectives
- Master core ROS 2 communication primitives
- Understand nodes as independent computational units
- Learn about topics for continuous data (sensors, state)
- Learn about services for discrete commands and queries
- Trace end-to-end data flow for a humanoid action

## Introduction

In the previous chapter, we explored ROS 2 as the nervous system of a robot. Now we'll dive deeper into the three fundamental communication mechanisms that make this nervous system work: nodes, topics, and services. These primitives form the backbone of all communication in ROS 2-based robotic systems.

## Core Concepts

### Nodes: Independent Computational Units

Nodes are the fundamental computational units in ROS 2. Each node runs independently and typically performs a specific function:

- **Specialization**: Each node focuses on a specific task (e.g., sensor processing, motion planning, control)
- **Isolation**: Nodes run in separate processes, so a failure in one node doesn't necessarily affect others
- **Communication**: Nodes interact with other nodes through topics and services

<div class="edu-concept">
  <strong>Key Concept:</strong> A node in ROS 2 is like a specialized neuron in a biological nervous system - it performs a specific function and communicates with other nodes through messages.
</div>

### Topics: Continuous Data Flow

Topics provide a publish-subscribe communication pattern for continuous data streams:

- **Unidirectional**: Data flows from publisher to subscriber
- **Many-to-many**: Multiple publishers can send to a topic, multiple subscribers can receive from it
- **Asynchronous**: Publishers and subscribers don't need to be synchronized in time
- **Best-effort delivery**: Messages may be lost, but delivery speed is prioritized

Common uses for topics:
- Sensor data (camera images, LIDAR scans, IMU readings)
- Robot state (position, velocity, joint angles)
- System status information

### Services: Discrete Request-Response Interactions

Services provide a request-response communication pattern for discrete interactions:

- **Bidirectional**: Request goes one way, response goes back
- **One-to-one**: One client requests from one server (though multiple clients can use the same service)
- **Synchronous**: The client typically waits for the response
- **Guaranteed delivery**: Requests and responses are reliably delivered

Common uses for services:
- Changing robot parameters
- Triggering specific actions
- Requesting specific computations

<div class="edu-example">
  <strong>Example:</strong> A path planning node might use topics to receive sensor data continuously, but use a service to request a specific path computation when needed.
</div>

## Biological Analogies

### Neural Communication Patterns

ROS 2 communication patterns have analogies in biological nervous systems:

- **Topics** are like neural pathways that continuously transmit sensory information (e.g., visual or tactile data)
- **Services** are like specific neural circuits that respond to discrete requests (e.g., reflex actions or decision-making processes)

## Practical Examples

### Example: Humanoid Walking

Consider a humanoid robot walking:

1. **Sensors publish to topics**:
   - IMU provides balance data
   - Joint encoders provide position feedback
   - Force sensors provide ground contact information

2. **Controllers subscribe to topics**:
   - Balance controller processes IMU data
   - Trajectory planner processes joint feedback
   - Gait controller processes force sensor data

3. **Services handle discrete commands**:
   - Request to start walking
   - Request to stop walking
   - Request to change walking speed

### End-to-End Data Flow for a Humanoid Action

Here's a complete example of a humanoid reaching for an object:

1. **Perception Node** publishes camera images to `/camera/image_raw` topic
2. **Object Detection Node** subscribes to `/camera/image_raw`, detects object, publishes position to `/object_position` topic
3. **Motion Planner Node** subscribes to `/object_position`, calculates reach trajectory, uses service `/compute_ik` to compute inverse kinematics
4. **Controller Node** receives trajectory and commands joints via action server (similar to service but for longer processes)

<div class="edu-analogy">
  <strong>Analogy:</strong> This flow is similar to how a person sees an object (sensory input), recognizes it (perception), decides how to reach it (planning), and executes the movement (control).
</div>

## Diagrams and Visuals

<div class="diagram-container">
  <img src="/img/nodes-topics-services.png" alt="Nodes, topics, and services communication patterns" />
  <div class="diagram-caption">Figure 2.1: Communication patterns in ROS 2: nodes, topics, and services</div>
</div>

## Summary

Nodes, topics, and services are the three fundamental communication primitives in ROS 2. Nodes are independent computational units that communicate through topics (for continuous data) and services (for discrete requests). Understanding these concepts is crucial for designing effective robotic systems.

## Exercises/Questions

1. When would you use a topic instead of a service? Provide specific examples.
2. What are the trade-offs between guaranteed delivery (services) and best-effort delivery (topics)?
3. Design a simple communication pattern for a robot that needs to avoid obstacles using sensor data.

## Next Steps

Continue to [Chapter 3: Python Agents, rclpy, and Humanoid Structure (URDF)](./03-python-agents-urdf.md) to learn about how Python AI agents interface with ROS controllers and how robot structure is represented.