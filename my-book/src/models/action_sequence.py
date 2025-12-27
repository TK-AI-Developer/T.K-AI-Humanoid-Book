"""
Action Sequence Model
Represents a series of executable commands for the robot to perform specific tasks
"""
from dataclasses import dataclass
from typing import List, Dict, Any, Optional
from uuid import uuid4
from datetime import datetime


@dataclass
class ActionStep:
    """A single action within an action sequence"""
    action_type: str  # 'move', 'rotate', 'grip', 'speak', etc.
    parameters: Dict[str, Any]
    description: Optional[str] = None
    estimated_duration: Optional[float] = None  # in seconds
    
    def validate(self) -> bool:
        """Validate that this action step is properly configured"""
        if not isinstance(self.action_type, str) or not self.action_type:
            return False
        if not isinstance(self.parameters, dict):
            return False
        if self.estimated_duration is not None and self.estimated_duration < 0:
            return False
        return True


@dataclass
class ActionSequence:
    """
    Series of executable commands for the robot to perform specific tasks
    """
    name: str
    sequence_id: str = None
    steps: List[ActionStep] = None
    parameters: Optional[Dict[str, Any]] = None
    execution_status: str = "pending"  # 'pending', 'in_progress', 'completed', 'failed', 'paused'
    estimated_duration: Optional[float] = None  # in seconds
    created_at: datetime = None
    updated_at: datetime = None
    
    def __post_init__(self):
        if self.sequence_id is None:
            self.sequence_id = f"seq_{uuid4().hex}"
        if self.steps is None:
            self.steps = []
        if self.parameters is None:
            self.parameters = {}
        if self.created_at is None:
            self.created_at = datetime.now()
        self.updated_at = datetime.now()
    
    def add_step(self, step: ActionStep) -> None:
        """Add an action step to the sequence"""
        if not step.validate():
            raise ValueError("Invalid action step")
        self.steps.append(step)
        self.updated_at = datetime.now()
        
        # Update estimated duration
        if self.estimated_duration is None:
            self.estimated_duration = 0
        if step.estimated_duration:
            self.estimated_duration += step.estimated_duration
    
    def remove_step(self, index: int) -> bool:
        """Remove an action step from the sequence by index"""
        if 0 <= index < len(self.steps):
            removed_step = self.steps.pop(index)
            self.updated_at = datetime.now()
            
            # Update estimated duration
            if self.estimated_duration is not None and removed_step.estimated_duration:
                self.estimated_duration -= removed_step.estimated_duration
            
            return True
        return False
    
    def update_status(self, new_status: str) -> None:
        """Update the execution status of the sequence"""
        valid_statuses = ["pending", "in_progress", "completed", "failed", "paused"]
        if new_status not in valid_statuses:
            raise ValueError(f"Invalid status: {new_status}. Valid statuses are: {valid_statuses}")
        
        self.execution_status = new_status
        self.updated_at = datetime.now()
    
    def validate(self) -> bool:
        """Validate that all steps are valid actions for the robot's capabilities"""
        # Check if name is valid
        if not self.name or not isinstance(self.name, str):
            return False
        
        # Check if all steps are valid
        for step in self.steps:
            if not isinstance(step, ActionStep) or not step.validate():
                return False
        
        # Check if parameters is a dictionary
        if not isinstance(self.parameters, dict):
            return False
        
        # Check if execution status is valid
        valid_statuses = ["pending", "in_progress", "completed", "failed", "paused"]
        if self.execution_status not in valid_statuses:
            return False
        
        # Check if estimated duration is valid
        if self.estimated_duration is not None and self.estimated_duration < 0:
            return False
        
        # Check if timestamps are valid
        if not isinstance(self.created_at, datetime) or not isinstance(self.updated_at, datetime):
            return False
        
        return True