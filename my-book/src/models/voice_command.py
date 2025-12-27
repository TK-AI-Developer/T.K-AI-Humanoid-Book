"""
Voice Command Model
Represents natural language input that needs to be processed and converted to actions
"""
from dataclasses import dataclass
from typing import Dict, Any, Optional
from uuid import uuid4
from datetime import datetime


@dataclass
class VoiceCommand:
    """
    Natural language input that needs to be processed and converted to actions
    """
    original_text: str
    command_id: str = None
    processed_text: Optional[str] = None
    intent: Optional[str] = None
    parameters: Optional[Dict[str, Any]] = None
    confidence_score: Optional[float] = None
    timestamp: datetime = None
    
    def __post_init__(self):
        if self.command_id is None:
            self.command_id = f"cmd_{uuid4().hex}"
        if self.processed_text is None:
            self.processed_text = self.original_text
        if self.parameters is None:
            self.parameters = {}
        if self.confidence_score is None:
            self.confidence_score = 1.0  # Default to high confidence if not specified
        if self.timestamp is None:
            self.timestamp = datetime.now()
    
    def update_processed_text(self, text: str) -> None:
        """Update the processed text"""
        self.processed_text = text
        self.timestamp = datetime.now()
    
    def update_intent(self, intent: str, confidence: float) -> None:
        """Update the identified intent and confidence score"""
        self.intent = intent
        self.confidence_score = confidence
        self.timestamp = datetime.now()
    
    def add_parameter(self, key: str, value: Any) -> None:
        """Add a parameter to the command"""
        self.parameters[key] = value
        self.timestamp = datetime.now()
    
    def validate(self) -> bool:
        """Validate that the command has a recognized intent and valid parameters"""
        # Check if original text is provided
        if not self.original_text or not isinstance(self.original_text, str):
            return False
        
        # Check if intent is provided and is a string
        if self.intent and not isinstance(self.intent, str):
            return False
        
        # Check if confidence score is valid
        if self.confidence_score is not None and not (0.0 <= self.confidence_score <= 1.0):
            return False
        
        # Check if parameters is a dictionary
        if not isinstance(self.parameters, dict):
            return False
        
        # Check if timestamp is valid
        if not isinstance(self.timestamp, datetime):
            return False
        
        return True