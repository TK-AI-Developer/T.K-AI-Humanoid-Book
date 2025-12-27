---
sidebar_position: 1
---

# Voice-to-Action

## Overview

This chapter covers converting voice commands to executable actions in simulation. Voice-to-action systems enable natural human-robot interaction by allowing users to communicate with robots using natural language. This chapter explores how to implement voice processing, natural language understanding, and action mapping in simulation environments.

## Learning Objectives

By the end of this chapter, you will be able to:
- Process voice commands using speech recognition
- Convert natural language to robot actions
- Implement intent recognition for voice commands
- Map voice commands to executable action sequences
- Evaluate voice-to-action system performance

## Voice-to-Action Pipeline

The voice-to-action system follows a pipeline that converts human speech into robot actions:

1. **Voice Input**: Capture voice command from user
2. **Speech-to-Text**: Convert voice to text using Whisper API
3. **Natural Language Understanding**: Parse text to identify intent and parameters
4. **Action Mapping**: Convert intent to specific robot action sequences
5. **Execution**: Execute actions in simulation environment

## Voice Command Model

The voice command is represented by the `VoiceCommand` model which includes:

- Original text of the command
- Processed text after normalization
- Identified intent from the command
- Extracted parameters
- Confidence score of intent recognition

```python
# Example of creating a voice command
from my_book.models.voice_command import VoiceCommand

# Create a voice command
command = VoiceCommand(
    original_text="Move the robot forward by 1 meter"
)

# Update the recognized intent
command.update_intent("move_robot", confidence=0.95)

# Add parameters extracted from the command
command.add_parameter("distance", 1.0)
command.add_parameter("direction", "forward")

# Validate the command
if command.validate():
    print("Voice command is valid")
else:
    print("Voice command validation failed")
```

## Implementing Voice Processing

In this implementation, we use the OpenAI Whisper API for speech-to-text conversion, followed by natural language processing to extract intent and parameters.

### Example: Processing a Voice Command

```python
# Example code for processing a voice command
from my_book.services.voice_service import VoiceService

# Initialize the voice service
voice_service = VoiceService()

# Process a voice command (in a real implementation, this would be audio data)
# For this example, we'll use text that simulates what Whisper would return
audio_data_simulation = "Move the robot forward by 1 meter"

command_id = voice_service.process_voice_command(
    audio_data=audio_data_simulation,
    language="en",
    robot_id="robot_12345"
)

print(f"Voice command processed with ID: {command_id}")
```

## Creating Action Sequences

Based on the recognized intent, the system creates executable action sequences for the robot.

### Example: Creating Action Sequence from Command

```python
# Create an action sequence based on the voice command
sequence_id = voice_service.create_action_sequence_from_command(command_id)

print(f"Action sequence created with ID: {sequence_id}")
```

## Cognitive Planning

For complex voice commands, the system creates cognitive plans that map natural language to action sequences.

### Example: Creating a Cognitive Plan

```python
# Create a cognitive plan for the voice command
plan_id = voice_service.create_cognitive_plan(command_id)

print(f"Cognitive plan created with ID: {plan_id}")
```

## Natural Language Understanding

The system uses intent recognition to understand the purpose of voice commands:

- **Move Robot**: Commands to move the robot in a specific direction
- **Rotate Robot**: Commands to rotate the robot
- **Manipulate Object**: Commands to interact with objects
- **Navigate to Location**: Commands to go to specific locations
- **Stop Robot**: Commands to stop robot operations

## Action Sequences

Action sequences are composed of individual action steps that the robot executes:

- **Move Base**: Move the robot to a specific location
- **Rotate Base**: Rotate the robot by a specific angle
- **Navigate to Object**: Move to a specific object
- **Grasp Object**: Grasp an object with the robot's gripper
- **Navigate to Location**: Move to a named location
- **Stop Robot**: Stop all robot movements
- **Speak**: Speak a text message

## Performance Evaluation

Evaluating voice-to-action systems involves metrics such as:

- Speech recognition accuracy
- Intent recognition accuracy
- Action mapping success rate
- Response latency
- User satisfaction

## Best Practices

- Use clear and consistent voice command formats
- Provide feedback to users about command recognition
- Implement error handling for unrecognized commands
- Test the system with various accents and speaking styles
- Validate actions before execution to prevent errors

## Summary

This chapter covered voice-to-action systems for human-robot interaction. You learned how to process voice commands, recognize intents, map commands to action sequences, and execute those actions in simulation. In the next chapter, we'll explore cognitive planning in more detail.