# API Contracts: AI-Robot Brain & Vision-Language-Action (VLA)

## Overview

This document defines the API contracts for the AI-Robot Brain & Vision-Language-Action implementation based on the functional requirements.

## Simulation Management API

### Create Simulation Environment
- **Endpoint**: `POST /api/simulation/environments`
- **Description**: Creates a new photorealistic simulation environment
- **Request**:
  ```json
  {
    "name": "string",
    "description": "string",
    "physics_properties": {
      "gravity": "number",
      "friction": "number"
    },
    "lighting_config": {
      "ambient_light": "number",
      "directional_light": {
        "intensity": "number",
        "direction": {"x": "number", "y": "number", "z": "number"}
      }
    },
    "objects": [
      {
        "type": "string",
        "position": {"x": "number", "y": "number", "z": "number"},
        "rotation": {"x": "number", "y": "number", "z": "number", "w": "number"}
      }
    ]
  }
  ```
- **Response**: 
  ```json
  {
    "environment_id": "string",
    "status": "created",
    "created_at": "timestamp"
  }
  ```

### Start Simulation
- **Endpoint**: `POST /api/simulation/{environment_id}/start`
- **Description**: Starts the simulation in the specified environment
- **Response**:
  ```json
  {
    "status": "running",
    "started_at": "timestamp"
  }
  ```

## Robot Management API

### Deploy Robot to Environment
- **Endpoint**: `POST /api/robots/deploy`
- **Description**: Deploys a humanoid robot to a simulation environment
- **Request**:
  ```json
  {
    "robot_model": "string",
    "environment_id": "string",
    "initial_position": {"x": "number", "y": "number", "z": "number"},
    "initial_orientation": {"x": "number", "y": "number", "z": "number", "w": "number"}
  }
  ```
- **Response**:
  ```json
  {
    "robot_id": "string",
    "status": "deployed",
    "deployed_at": "timestamp"
  }
  ```

### Get Robot Status
- **Endpoint**: `GET /api/robots/{robot_id}/status`
- **Description**: Retrieves the current status of the robot
- **Response**:
  ```json
  {
    "robot_id": "string",
    "status": "string",
    "position": {"x": "number", "y": "number", "z": "number"},
    "orientation": {"x": "number", "y": "number", "z": "number", "w": "number"},
    "battery_level": "number",
    "last_updated": "timestamp"
  }
  ```

## Perception API

### Get Perception Data
- **Endpoint**: `GET /api/robots/{robot_id}/perception`
- **Description**: Retrieves perception data from robot sensors
- **Response**:
  ```json
  {
    "robot_id": "string",
    "timestamp": "timestamp",
    "sensor_data": {
      "camera": {
        "image_data": "base64",
        "resolution": {"width": "number", "height": "number"}
      },
      "lidar": {
        "point_cloud": [{"x": "number", "y": "number", "z": "number"}],
        "range": "number"
      }
    },
    "environment_state": {
      "objects": [
        {
          "id": "string",
          "type": "string",
          "position": {"x": "number", "y": "number", "z": "number"}
        }
      ],
      "obstacles": [
        {
          "id": "string",
          "position": {"x": "number", "y": "number", "z": "number"},
          "size": {"x": "number", "y": "number", "z": "number"}
        }
      ]
    }
  }
  ```

## Navigation API

### Plan Path
- **Endpoint**: `POST /api/robots/{robot_id}/navigation/plan`
- **Description**: Plans a navigation path for the robot
- **Request**:
  ```json
  {
    "destination": {"x": "number", "y": "number", "z": "number"},
    "path_type": "optimal|safe|fast",
    "avoid_dynamic_obstacles": "boolean"
  }
  ```
- **Response**:
  ```json
  {
    "path_id": "string",
    "waypoints": [
      {"x": "number", "y": "number", "z": "number"}
    ],
    "estimated_duration": "number",
    "distance": "number"
  }
  ```

### Execute Navigation
- **Endpoint**: `POST /api/robots/{robot_id}/navigation/execute`
- **Description**: Executes a planned navigation path
- **Request**:
  ```json
  {
    "path_id": "string",
    "speed": "number"
  }
  ```
- **Response**:
  ```json
  {
    "status": "executing",
    "estimated_completion": "timestamp"
  }
  ```

## Voice Command API

### Process Voice Command
- **Endpoint**: `POST /api/voice/process`
- **Description**: Processes a voice command and converts it to text
- **Request**:
  ```json
  {
    "audio_data": "base64",
    "language": "string",
    "robot_id": "string"
  }
  ```
- **Response**:
  ```json
  {
    "command_id": "string",
    "original_text": "string",
    "processed_text": "string",
    "intent": "string",
    "parameters": "object",
    "confidence_score": "number"
  }
  ```

## Action Execution API

### Execute Action Sequence
- **Endpoint**: `POST /api/robots/{robot_id}/actions/execute`
- **Description**: Executes a sequence of actions on the robot
- **Request**:
  ```json
  {
    "sequence_id": "string",
    "actions": [
      {
        "action_type": "string",
        "parameters": "object"
      }
    ]
  }
  ```
- **Response**:
  ```json
  {
    "execution_id": "string",
    "status": "executing",
    "estimated_completion": "timestamp"
  }
  ```

### Get Execution Status
- **Endpoint**: `GET /api/execution/{execution_id}/status`
- **Description**: Gets the status of an action sequence execution
- **Response**:
  ```json
  {
    "execution_id": "string",
    "status": "pending|in_progress|completed|failed|paused",
    "current_step": "number",
    "total_steps": "number",
    "completed_at": "timestamp|null"
  }
  ```