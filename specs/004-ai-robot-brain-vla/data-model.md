# Data Model: AI-Robot Brain & Vision-Language-Action (VLA) Implementation

## Overview

This document defines the key data structures and entities for the AI-Robot Brain & Vision-Language-Action implementation, focusing on simulation, perception, navigation, and voice-driven actions.

## Core Entities

### Simulation Environment
- **Name**: Virtual world with physics, lighting and objects for robot training and testing
- **Attributes**:
  - environment_id: unique identifier for the simulation environment
  - name: descriptive name of the environment
  - description: detailed description of the environment
  - physics_properties: parameters for physics simulation (gravity, friction, etc.)
  - lighting_config: lighting setup for photorealistic rendering
  - objects: list of static and dynamic objects in the environment
  - sensors: list of available sensors in the environment
- **Relationships**: Contains multiple Humanoid Robot instances
- **Validation**: Must have valid physics properties and lighting configuration

### Humanoid Robot
- **Name**: Virtual bipedal robot with sensors, actuators and control systems
- **Attributes**:
  - robot_id: unique identifier for the robot
  - model_name: name of the robot model
  - bipedal_config: configuration for bipedal movement
  - sensors: list of attached sensors (cameras, lidar, etc.)
  - actuators: list of available actuators (motors, effectors)
  - current_position: 3D coordinates in the environment
  - current_orientation: 3D orientation in the environment
  - status: current operational status (idle, moving, executing task, etc.)
- **Relationships**: Belongs to a Simulation Environment, executes Action Sequences
- **Validation**: Must have valid sensor and actuator configurations

### Voice Command
- **Name**: Natural language input that needs to be processed and converted to actions
- **Attributes**:
  - command_id: unique identifier for the command
  - original_text: the original voice command as text
  - processed_text: the processed and normalized text
  - intent: the identified intent from the command
  - parameters: extracted parameters from the command
  - confidence_score: confidence level of intent recognition
  - timestamp: when the command was received
- **Relationships**: Maps to Action Sequence
- **Validation**: Must have a recognized intent and valid parameters

### Action Sequence
- **Name**: Series of executable commands for the robot to perform specific tasks
- **Attributes**:
  - sequence_id: unique identifier for the sequence
  - name: descriptive name of the sequence
  - steps: ordered list of individual actions
  - parameters: parameters for the entire sequence
  - execution_status: current status of sequence execution
  - estimated_duration: estimated time to complete the sequence
- **Relationships**: Executed by Humanoid Robot, originates from Voice Command
- **Validation**: All steps must be valid actions for the robot's capabilities

## Supporting Entities

### Perception Data
- **Name**: Data collected by robot sensors during operation
- **Attributes**:
  - data_id: unique identifier for the data
  - robot_id: reference to the collecting robot
  - sensor_type: type of sensor that collected the data
  - timestamp: when the data was collected
  - raw_data: raw sensor data
  - processed_data: processed sensor data
  - environment_state: interpretation of the environment from the data
- **Relationships**: Collected by Humanoid Robot
- **Validation**: Raw data must match the sensor type format

### Navigation Path
- **Name**: Planned route for robot movement in the environment
- **Attributes**:
  - path_id: unique identifier for the path
  - start_position: starting coordinates
  - end_position: destination coordinates
  - waypoints: ordered list of intermediate positions
  - path_type: type of path (optimal, safe, fast, etc.)
  - obstacle_avoidance: information about how obstacles were avoided
  - estimated_duration: estimated time to traverse the path
- **Relationships**: Associated with Humanoid Robot, exists within Simulation Environment
- **Validation**: Waypoints must be reachable and collision-free

### Cognitive Plan
- **Name**: High-level plan mapping natural language to action sequences
- **Attributes**:
  - plan_id: unique identifier for the plan
  - input_command: original natural language command
  - breakdown: step-by-step breakdown of the plan
  - action_sequences: ordered list of action sequences needed
  - context: relevant context for plan execution
  - success_criteria: conditions for plan completion
- **Relationships**: Generated from Voice Command, executed as Action Sequences
- **Validation**: Must have a clear path to achieve success criteria

## State Transitions

### Humanoid Robot States
- IDLE → NAVIGATING (when navigation command is received)
- NAVIGATING → EXECUTING_TASK (when reaching destination and starting task)
- EXECUTING_TASK → IDLE (when task is completed)
- Any state → EMERGENCY_STOP (when safety conditions are met)

### Action Sequence States
- PENDING → IN_PROGRESS (when execution starts)
- IN_PROGRESS → COMPLETED (when all steps are executed)
- IN_PROGRESS → FAILED (when execution fails)
- IN_PROGRESS → PAUSED (when execution is paused)

## Data Relationships

```
Simulation Environment (1) -- (Many) Humanoid Robot
Humanoid Robot (1) -- (Many) Perception Data
Voice Command (1) -- (1) Action Sequence
Action Sequence (1) -- (Many) Individual Actions
Humanoid Robot (1) -- (Many) Navigation Path
Voice Command (1) -- (1) Cognitive Plan
Cognitive Plan (1) -- (Many) Action Sequences
```