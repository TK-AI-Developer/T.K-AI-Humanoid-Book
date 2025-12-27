---
sidebar_label: 'Module 1: ROS 2 as the Nervous System'
---

# Module 1: ROS 2 as the Nervous System of Humanoid Robots

Welcome to Module 1 of the Physical AI & Humanoid Robotics course. This module establishes ROS 2 as the foundational middleware that connects AI decision-making to physical humanoid bodies. This module forms the conceptual and architectural base for all simulation, perception, and control work in later modules.

## Learning Path

This module consists of three chapters that build upon each other:

### [Chapter 1: ROS 2 as the Nervous System of a Robot](./01-ros2-nervous-system.md)
- Understand why physical robots require middleware
- Learn how ROS 2 functions as a distributed, message-driven system
- Explore analogies between biological nervous systems and ROS concepts
- Understand reliability, modularity, and real-world constraints

### [Chapter 2: Nodes, Topics, and Services](./02-nodes-topics-services.md)
- Master core ROS 2 communication primitives
- Understand nodes as independent computational units
- Learn about topics for continuous data (sensors, state)
- Learn about services for discrete commands and queries
- Trace end-to-end data flow for a humanoid action

### [Chapter 3: Python Agents, rclpy, and Humanoid Structure (URDF)](./03-python-agents-urdf.md)
- Understand the role of rclpy in Python-based AI control
- Learn the separation of decision logic and motor execution
- Understand URDF as a structural model of humanoids
- Learn about links, joints, and kinematic hierarchy

## Prerequisites

- Basic Python programming knowledge
- Familiarity with AI/ML concepts
- Understanding of basic robotics concepts (optional but helpful)

## Learning Outcomes

After completing this module, you will be able to:
- Explain ROS 2 as a distributed nervous system for humanoid robots
- Reason about nodes, topics, and services in real robot workflows
- Understand how Python AI agents interface with ROS controllers
- Interpret humanoid robot structure using URDF