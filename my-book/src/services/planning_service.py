"""
Planning Service
Implements natural language to action mapping logic
"""
from typing import Dict, Any
from .models.action_sequence import ActionSequence, ActionStep
from .models.voice_command import VoiceCommand
import uuid


class PlanningService:
    """
    Service for creating natural language to action mapping logic
    """
    
    def __init__(self):
        self.action_sequences: Dict[str, ActionSequence] = {}
    
    def map_natural_language_to_action(self, voice_command: VoiceCommand) -> str:
        """
        Create an action sequence based on the voice command
        """
        # Create an action sequence based on the recognized intent
        sequence = ActionSequence(
            name=f"Sequence for: {voice_command.original_text}",
            parameters={"command_id": voice_command.command_id}
        )
        
        # Map intents to action sequences
        if voice_command.intent == "move_robot":
            # Create action steps for moving the robot
            move_step = ActionStep(
                action_type="move_base",
                parameters={
                    "direction": "forward",
                    "distance": 1.0,  # meter
                    "speed": 0.5  # m/s
                },
                description="Move the robot forward by 1 meter"
            )
            sequence.add_step(move_step)
        
        elif voice_command.intent == "rotate_robot":
            # Create action steps for rotating the robot
            rotate_step = ActionStep(
                action_type="rotate_base",
                parameters={
                    "angle": 90.0,  # degrees
                    "direction": "left"
                },
                description="Rotate the robot left by 90 degrees"
            )
            sequence.add_step(rotate_step)
        
        elif voice_command.intent == "manipulate_object":
            # Create action steps for manipulating an object
            navigate_step = ActionStep(
                action_type="navigate_to_object",
                parameters={
                    "object_color": "red",
                    "object_type": "cube"
                },
                description="Navigate to the red cube"
            )
            sequence.add_step(navigate_step)
            
            grasp_step = ActionStep(
                action_type="grasp_object",
                parameters={
                    "object_id": "red_cube_1"
                },
                description="Grasp the red cube"
            )
            sequence.add_step(grasp_step)
        
        elif voice_command.intent == "navigate_to_location":
            # Create action steps for navigating to a location
            navigate_step = ActionStep(
                action_type="navigate_to_location",
                parameters={
                    "location": "kitchen"
                },
                description="Navigate to the kitchen area"
            )
            sequence.add_step(navigate_step)
        
        elif voice_command.intent == "stop_robot":
            # Create action steps for stopping the robot
            stop_step = ActionStep(
                action_type="stop_robot",
                parameters={},
                description="Stop all robot movements"
            )
            sequence.add_step(stop_step)
        
        else:
            # Unknown command - create a speak action to indicate error
            speak_step = ActionStep(
                action_type="speak",
                parameters={
                    "text": "I did not understand that command"
                },
                description="Speak an error message"
            )
            sequence.add_step(speak_step)
        
        # Validate and store the sequence
        if not sequence.validate():
            raise ValueError("Invalid action sequence")
        
        self.action_sequences[sequence.sequence_id] = sequence
        return sequence.sequence_id