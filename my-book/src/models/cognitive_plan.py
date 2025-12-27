"""
Cognitive Plan Model
Represents a high-level plan mapping natural language to action sequences
"""
from dataclasses import dataclass
from typing import List, Dict, Any, Optional
from uuid import uuid4
from datetime import datetime
from .action_sequence import ActionSequence


@dataclass
class PlanStep:
    """A single step in a cognitive plan"""
    description: str
    action_sequence_id: Optional[str] = None
    dependencies: Optional[List[str]] = None  # IDs of other steps this step depends on
    estimated_duration: Optional[float] = None  # in seconds
    
    def __post_init__(self):
        if self.dependencies is None:
            self.dependencies = []


@dataclass
class CognitivePlan:
    """
    High-level plan mapping natural language to action sequences
    """
    input_command: str
    plan_id: str = None
    breakdown: Optional[List[PlanStep]] = None
    action_sequences: Optional[List[ActionSequence]] = None
    context: Optional[Dict[str, Any]] = None
    success_criteria: Optional[List[str]] = None
    created_at: datetime = None
    updated_at: datetime = None
    
    def __post_init__(self):
        if self.plan_id is None:
            self.plan_id = f"plan_{uuid4().hex}"
        if self.breakdown is None:
            self.breakdown = []
        if self.action_sequences is None:
            self.action_sequences = []
        if self.context is None:
            self.context = {}
        if self.success_criteria is None:
            self.success_criteria = []
        if self.created_at is None:
            self.created_at = datetime.now()
        self.updated_at = datetime.now()
    
    def add_step(self, step: PlanStep) -> None:
        """Add a step to the plan breakdown"""
        self.breakdown.append(step)
        self.updated_at = datetime.now()
    
    def add_action_sequence(self, sequence: ActionSequence) -> None:
        """Add an action sequence to the plan"""
        if not sequence.validate():
            raise ValueError("Invalid action sequence")
        self.action_sequences.append(sequence)
        self.updated_at = datetime.now()
    
    def add_success_criteria(self, criteria: str) -> None:
        """Add a success criterion to the plan"""
        self.success_criteria.append(criteria)
        self.updated_at = datetime.now()
    
    def validate(self) -> bool:
        """Validate that the plan has a clear path to achieve success criteria"""
        # Check if input command is provided
        if not self.input_command or not isinstance(self.input_command, str):
            return False
        
        # Check if breakdown is a list
        if not isinstance(self.breakdown, list):
            return False
        
        # Validate each step in the breakdown
        for step in self.breakdown:
            if not isinstance(step, PlanStep):
                return False
            if not step.description:
                return False
        
        # Check if action sequences are valid
        for sequence in self.action_sequences:
            if not isinstance(sequence, ActionSequence) or not sequence.validate():
                return False
        
        # Check if context is a dictionary
        if not isinstance(self.context, dict):
            return False
        
        # Check if success criteria is a list of strings
        if not isinstance(self.success_criteria, list):
            return False
        for criteria in self.success_criteria:
            if not isinstance(criteria, str):
                return False
        
        # Check if timestamps are valid
        if not isinstance(self.created_at, datetime) or not isinstance(self.updated_at, datetime):
            return False
        
        return True