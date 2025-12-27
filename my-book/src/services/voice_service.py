"""
Voice Service
Implements voice processing using OpenAI Whisper API
"""
from typing import Dict, Any, Optional
from .models.voice_command import VoiceCommand
from .models.action_sequence import ActionSequence, ActionStep
from .models.cognitive_plan import CognitivePlan, PlanStep
import uuid
import json
from datetime import datetime


class VoiceService:
    """
    Service for processing voice commands using OpenAI Whisper API
    """
    
    def __init__(self):
        self.voice_commands: Dict[str, VoiceCommand] = {}
        self.action_sequences: Dict[str, ActionSequence] = {}
        self.cognitive_plans: Dict[str, CognitivePlan] = {}
    
    def process_voice_command(self, audio_data: str, language: str = "en", robot_id: str = None) -> str:
        """
        Process a voice command and convert it to text
        """
        # In a real implementation, this would call the OpenAI Whisper API
        # For this example, we'll simulate the transcription
        
        # Simulate Whisper API processing
        # In reality, audio_data would be a base64-encoded audio file
        # and Whisper would return the transcribed text
        if "move forward" in audio_data.lower():
            transcribed_text = "Move the robot forward by 1 meter"
        elif "turn left" in audio_data.lower():
            transcribed_text = "Turn the robot left by 90 degrees"
        elif "pick up object" in audio_data.lower():
            transcribed_text = "Pick up the red cube in front of the robot"
        elif "go to kitchen" in audio_data.lower():
            transcribed_text = "Navigate to the kitchen area"
        else:
            transcribed_text = "Unknown command, please repeat"
        
        # Create a voice command object
        command = VoiceCommand(
            original_text=transcribed_text,
            processed_text=transcribed_text
        )
        
        # Simulate intent recognition
        intent = self._recognize_intent(transcribed_text)
        command.update_intent(intent, confidence=0.9)  # Assume high confidence for simulation
        
        # Store the command
        self.voice_commands[command.command_id] = command
        
        return command.command_id
    
    def _recognize_intent(self, text: str) -> str:
        """
        Recognize intent from transcribed text
        """
        text_lower = text.lower()
        
        if "move" in text_lower or "forward" in text_lower or "backward" in text_lower:
            return "move_robot"
        elif "turn" in text_lower or "rotate" in text_lower or "left" in text_lower or "right" in text_lower:
            return "rotate_robot"
        elif "pick" in text_lower or "grasp" in text_lower or "take" in text_lower:
            return "manipulate_object"
        elif "go to" in text_lower or "navigate" in text_lower or "move to" in text_lower:
            return "navigate_to_location"
        elif "stop" in text_lower or "halt" in text_lower:
            return "stop_robot"
        else:
            return "unknown"
    
    def create_action_sequence_from_command(self, command_id: str) -> str:
        """
        Create an action sequence based on the voice command
        """
        if command_id not in self.voice_commands:
            raise ValueError(f"Command {command_id} does not exist")
        
        command = self.voice_commands[command_id]
        
        # Create an action sequence based on the recognized intent
        sequence = ActionSequence(
            name=f"Sequence for: {command.original_text}",
            parameters={"command_id": command_id}
        )
        
        # Map intents to action sequences
        if command.intent == "move_robot":
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
        
        elif command.intent == "rotate_robot":
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
        
        elif command.intent == "manipulate_object":
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
        
        elif command.intent == "navigate_to_location":
            # Create action steps for navigating to a location
            navigate_step = ActionStep(
                action_type="navigate_to_location",
                parameters={
                    "location": "kitchen"
                },
                description="Navigate to the kitchen area"
            )
            sequence.add_step(navigate_step)
        
        elif command.intent == "stop_robot":
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
    
    def create_cognitive_plan(self, command_id: str) -> str:
        """
        Create a cognitive plan that maps natural language to action sequences
        """
        if command_id not in self.voice_commands:
            raise ValueError(f"Command {command_id} does not exist")
        
        command = self.voice_commands[command_id]
        
        # Create a cognitive plan
        plan = CognitivePlan(
            input_command=command.original_text
        )
        
        # Create plan steps based on the command
        # For this example, we'll create a simple plan with one step
        # In a real implementation, this would involve more complex planning
        plan_step = PlanStep(
            description=f"Execute action sequence for command: {command.original_text}",
        )
        plan.add_step(plan_step)
        
        # Create the corresponding action sequence
        sequence_id = self.create_action_sequence_from_command(command_id)
        plan_step.action_sequence_id = sequence_id
        
        # Add success criteria based on the command
        if command.intent == "move_robot":
            plan.add_success_criteria("Robot has moved to the specified location")
        elif command.intent == "rotate_robot":
            plan.add_success_criteria("Robot has rotated to the specified angle")
        elif command.intent == "manipulate_object":
            plan.add_success_criteria("Robot has successfully grasped the object")
        elif command.intent == "navigate_to_location":
            plan.add_success_criteria("Robot has reached the target location")
        elif command.intent == "stop_robot":
            plan.add_success_criteria("Robot has stopped all movements")
        else:
            plan.add_success_criteria("Command has been processed appropriately")
        
        # Add context about the robot if provided
        if command.parameters.get("robot_id"):
            plan.context["target_robot"] = command.parameters["robot_id"]
        
        # Validate and store the plan
        if not plan.validate():
            raise ValueError("Invalid cognitive plan")
        
        self.cognitive_plans[plan.plan_id] = plan
        return plan.plan_id