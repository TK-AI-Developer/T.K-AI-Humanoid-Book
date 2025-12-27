"""
Simulation Environment Model
Represents a virtual world with physics, lighting and objects for robot training and testing
"""
from dataclasses import dataclass
from typing import List, Dict, Any, Optional
from uuid import uuid4
from datetime import datetime


@dataclass
class PhysicsProperties:
    """Physics simulation parameters"""
    gravity: float = 9.81  # m/s^2
    friction: float = 0.5  # dimensionless coefficient
    time_step: float = 0.01  # seconds


@dataclass
class LightingConfig:
    """Lighting setup for photorealistic rendering"""
    ambient_light: float = 1.0
    directional_light: Dict[str, Any] = None
    
    def __post_init__(self):
        if self.directional_light is None:
            self.directional_light = {
                "intensity": 1.0,
                "direction": {"x": -1.0, "y": -1.0, "z": -1.0}
            }


@dataclass
class SimulationObject:
    """An object in the simulation environment"""
    object_type: str
    position: Dict[str, float]  # {"x": float, "y": float, "z": float}
    rotation: Dict[str, float]  # {"x": float, "y": float, "z": float, "w": float} - quaternion
    name: Optional[str] = None
    scale: Dict[str, float] = None
    
    def __post_init__(self):
        if self.scale is None:
            self.scale = {"x": 1.0, "y": 1.0, "z": 1.0}
        if self.name is None:
            self.name = f"{self.object_type}_{uuid4().hex[:8]}"


@dataclass
class SimulationEnvironment:
    """
    Virtual world with physics, lighting and objects for robot training and testing
    """
    name: str
    description: str
    physics_properties: PhysicsProperties
    lighting_config: LightingConfig
    environment_id: str = None
    objects: List[SimulationObject] = None
    sensors: List[str] = None  # List of sensor types available in this environment
    created_at: datetime = None
    updated_at: datetime = None
    
    def __post_init__(self):
        if self.environment_id is None:
            self.environment_id = f"env_{uuid4().hex}"
        if self.objects is None:
            self.objects = []
        if self.sensors is None:
            self.sensors = []
        if self.created_at is None:
            self.created_at = datetime.now()
        self.updated_at = datetime.now()
    
    def add_object(self, sim_object: SimulationObject) -> None:
        """Add an object to the simulation environment"""
        self.objects.append(sim_object)
        self.updated_at = datetime.now()
    
    def remove_object(self, object_name: str) -> bool:
        """Remove an object from the simulation environment by name"""
        for i, obj in enumerate(self.objects):
            if obj.name == object_name:
                del self.objects[i]
                self.updated_at = datetime.now()
                return True
        return False
    
    def validate(self) -> bool:
        """Validate that the environment has valid physics properties and lighting configuration"""
        # Check physics properties
        if not isinstance(self.physics_properties, PhysicsProperties):
            return False
        if not (0.1 <= self.physics_properties.gravity <= 20.0):  # Reasonable gravity range
            return False
        if not (0.0 <= self.physics_properties.friction <= 10.0):  # Reasonable friction range
            return False
        
        # Check lighting configuration
        if not isinstance(self.lighting_config, LightingConfig):
            return False
        if not (0.0 <= self.lighting_config.ambient_light <= 5.0):  # Reasonable ambient light range
            return False
        
        return True