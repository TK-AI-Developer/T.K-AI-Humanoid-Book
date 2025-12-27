"""
Navigation Path Model
Represents a planned route for robot movement in the environment
"""
from dataclasses import dataclass
from typing import List, Dict, Any, Optional
from uuid import uuid4
from datetime import datetime


@dataclass
class Waypoint:
    """A point in the navigation path"""
    position: Dict[str, float]  # {"x": float, "y": float, "z": float}
    description: Optional[str] = None
    timestamp: Optional[datetime] = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now()


@dataclass
class NavigationPath:
    """
    Planned route for robot movement in the environment
    """
    start_position: Dict[str, float]  # {"x": float, "y": float, "z": float}
    end_position: Dict[str, float]    # {"x": float, "y": float, "z": float}
    path_id: str = None
    waypoints: List[Waypoint] = None
    path_type: str = "optimal"  # "optimal", "safe", "fast"
    obstacle_avoidance: Optional[Dict[str, Any]] = None
    estimated_duration: Optional[float] = None  # in seconds
    distance: Optional[float] = None  # in meters
    created_at: datetime = None
    updated_at: datetime = None
    
    def __post_init__(self):
        if self.path_id is None:
            self.path_id = f"path_{uuid4().hex}"
        if self.waypoints is None:
            self.waypoints = []
        if self.obstacle_avoidance is None:
            self.obstacle_avoidance = {
                "enabled": True,
                "detection_range": 5.0,  # meters
                "safety_buffer": 0.5    # meters
            }
        if self.created_at is None:
            self.created_at = datetime.now()
        self.updated_at = datetime.now()
    
    def add_waypoint(self, waypoint: Waypoint) -> None:
        """Add a waypoint to the navigation path"""
        self.waypoints.append(waypoint)
        self.updated_at = datetime.now()
    
    def remove_waypoint(self, index: int) -> bool:
        """Remove a waypoint from the navigation path by index"""
        if 0 <= index < len(self.waypoints):
            del self.waypoints[index]
            self.updated_at = datetime.now()
            return True
        return False
    
    def calculate_distance(self) -> float:
        """Calculate the total distance of the path"""
        if not self.waypoints:
            # If no waypoints, return direct distance between start and end
            dx = self.end_position["x"] - self.start_position["x"]
            dy = self.end_position["y"] - self.start_position["y"]
            dz = self.end_position["z"] - self.start_position["z"]
            return (dx**2 + dy**2 + dz**2)**0.5
        
        # Calculate distance between consecutive waypoints
        total_distance = 0.0
        prev_pos = self.start_position
        for waypoint in self.waypoints:
            dx = waypoint.position["x"] - prev_pos["x"]
            dy = waypoint.position["y"] - prev_pos["y"]
            dz = waypoint.position["z"] - prev_pos["z"]
            total_distance += (dx**2 + dy**2 + dz**2)**0.5
            prev_pos = waypoint.position
        
        # Add distance from last waypoint to end
        dx = self.end_position["x"] - prev_pos["x"]
        dy = self.end_position["y"] - prev_pos["y"]
        dz = self.end_position["z"] - prev_pos["z"]
        total_distance += (dx**2 + dy**2 + dz**2)**0.5
        
        return total_distance
    
    def validate(self) -> bool:
        """Validate that waypoints are reachable and collision-free"""
        # Check if start and end positions are valid
        for pos_key in ["x", "y", "z"]:
            if not isinstance(self.start_position.get(pos_key), (int, float)):
                return False
            if not isinstance(self.end_position.get(pos_key), (int, float)):
                return False
        
        # Check if waypoints are valid
        for waypoint in self.waypoints:
            if not isinstance(waypoint, Waypoint):
                return False
            for pos_key in ["x", "y", "z"]:
                if not isinstance(waypoint.position.get(pos_key), (int, float)):
                    return False
        
        # Check path type is valid
        valid_path_types = ["optimal", "safe", "fast"]
        if self.path_type not in valid_path_types:
            return False
        
        # Check obstacle avoidance settings
        if not isinstance(self.obstacle_avoidance, dict):
            return False
        if "enabled" not in self.obstacle_avoidance:
            return False
        if not isinstance(self.obstacle_avoidance["enabled"], bool):
            return False
        
        return True