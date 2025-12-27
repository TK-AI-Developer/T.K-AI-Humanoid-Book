"""
Perception Data Model
Represents data collected by robot sensors during operation
"""
from dataclasses import dataclass
from typing import Dict, Any, Optional, List
from uuid import uuid4
from datetime import datetime


@dataclass
class CameraData:
    """Camera sensor data"""
    image_data: str  # base64 encoded image
    resolution: Dict[str, int]  # {"width": int, "height": int}
    timestamp: datetime = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now()


@dataclass
class LidarData:
    """LiDAR sensor data"""
    point_cloud: List[Dict[str, float]]  # [{"x": float, "y": float, "z": float}, ...]
    range: float  # max range in meters
    timestamp: datetime = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now()


@dataclass
class EnvironmentObject:
    """An object detected in the environment"""
    object_id: str
    object_type: str
    position: Dict[str, float]  # {"x": float, "y": float, "z": float}
    timestamp: datetime = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now()
        if self.object_id is None:
            self.object_id = f"obj_{uuid4().hex}"


@dataclass
class Obstacle:
    """An obstacle detected in the environment"""
    obstacle_id: str
    position: Dict[str, float]  # {"x": float, "y": float, "z": float}
    size: Dict[str, float]  # {"x": float, "y": float, "z": float}
    timestamp: datetime = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now()
        if self.obstacle_id is None:
            self.obstacle_id = f"obs_{uuid4().hex}"


@dataclass
class EnvironmentState:
    """Interpretation of the environment from sensor data"""
    objects: List[EnvironmentObject]
    obstacles: List[Obstacle]
    timestamp: datetime = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now()
        if self.objects is None:
            self.objects = []
        if self.obstacles is None:
            self.obstacles = []


@dataclass
class PerceptionData:
    """
    Data collected by robot sensors during operation
    """
    robot_id: str
    data_id: str = None
    sensor_type: str = None  # 'camera', 'lidar', 'imu', etc.
    timestamp: datetime = None
    raw_data: Optional[Dict[str, Any]] = None
    processed_data: Optional[Dict[str, Any]] = None
    environment_state: Optional[EnvironmentState] = None
    
    def __post_init__(self):
        if self.data_id is None:
            self.data_id = f"data_{uuid4().hex}"
        if self.timestamp is None:
            self.timestamp = datetime.now()
        if self.raw_data is None:
            self.raw_data = {}
        if self.processed_data is None:
            self.processed_data = {}
        if self.environment_state is None:
            self.environment_state = EnvironmentState(objects=[], obstacles=[])
    
    def add_camera_data(self, camera_data: CameraData) -> None:
        """Add camera sensor data to raw data"""
        self.raw_data["camera"] = {
            "image_data": camera_data.image_data,
            "resolution": camera_data.resolution,
            "timestamp": camera_data.timestamp.isoformat()
        }
        self.timestamp = datetime.now()
    
    def add_lidar_data(self, lidar_data: LidarData) -> None:
        """Add LiDAR sensor data to raw data"""
        self.raw_data["lidar"] = {
            "point_cloud": lidar_data.point_cloud,
            "range": lidar_data.range,
            "timestamp": lidar_data.timestamp.isoformat()
        }
        self.timestamp = datetime.now()
    
    def process_sensor_data(self) -> None:
        """Process raw sensor data to extract environment state"""
        # This is a simplified example - in a real implementation,
        # this would involve complex computer vision and sensor fusion algorithms
        processed = {}
        
        # Process camera data if available
        if "camera" in self.raw_data:
            # Extract objects from camera data (simplified)
            processed["camera_objects"] = []
            # In a real implementation, object detection would happen here
        
        # Process LiDAR data if available
        if "lidar" in self.raw_data:
            lidar_data = self.raw_data["lidar"]
            # Extract obstacles from LiDAR data (simplified)
            processed["lidar_obstacles"] = []
            # In a real implementation, obstacle detection would happen here
        
        self.processed_data = processed
        self.timestamp = datetime.now()
    
    def validate(self) -> bool:
        """Validate that raw data matches the sensor type format"""
        if not self.robot_id:
            return False
        
        # Validate camera data if sensor type is camera
        if self.sensor_type == "camera":
            if "camera" not in self.raw_data:
                return False
            camera_data = self.raw_data["camera"]
            if not isinstance(camera_data.get("image_data"), str):
                return False
            resolution = camera_data.get("resolution")
            if not isinstance(resolution, dict):
                return False
            if not all(isinstance(resolution.get(key), int) for key in ["width", "height"]):
                return False
        
        # Validate LiDAR data if sensor type is lidar
        elif self.sensor_type == "lidar":
            if "lidar" not in self.raw_data:
                return False
            lidar_data = self.raw_data["lidar"]
            if not isinstance(lidar_data.get("point_cloud"), list):
                return False
            if not isinstance(lidar_data.get("range"), (int, float)):
                return False
            # Validate point cloud format
            for point in lidar_data["point_cloud"]:
                if not all(isinstance(point.get(key), (int, float)) for key in ["x", "y", "z"]):
                    return False
        
        # Validate environment state
        if not isinstance(self.environment_state, EnvironmentState):
            return False
        
        # Validate objects in environment state
        for obj in self.environment_state.objects:
            if not isinstance(obj, EnvironmentObject):
                return False
            if not all(isinstance(obj.position.get(key), (int, float)) for key in ["x", "y", "z"]):
                return False
        
        # Validate obstacles in environment state
        for obs in self.environment_state.obstacles:
            if not isinstance(obs, Obstacle):
                return False
            if not all(isinstance(obs.position.get(key), (int, float)) for key in ["x", "y", "z"]):
                return False
            if not all(isinstance(obs.size.get(key), (int, float)) for key in ["x", "y", "z"]):
                return False
        
        return True