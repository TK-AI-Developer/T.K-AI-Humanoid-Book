"""
Humanoid Robot Model
Represents a virtual bipedal robot with sensors, actuators and control systems
"""
from dataclasses import dataclass
from typing import List, Dict, Any, Optional
from uuid import uuid4
from datetime import datetime


@dataclass
class BipedalConfig:
    """Configuration for bipedal movement"""
    leg_length: float  # meters
    hip_width: float   # meters
    max_step_size: float  # meters
    max_angular_velocity: float  # radians per second
    balance_threshold: float = 0.1  # angle in radians that triggers balance correction


@dataclass
class Sensor:
    """A sensor attached to the robot"""
    sensor_type: str  # 'camera', 'lidar', 'imu', 'gyroscope', etc.
    position: Dict[str, float]  # {"x": float, "y": float, "z": float}
    orientation: Dict[str, float]  # {"x": float, "y": float, "z": float, "w": float} - quaternion
    name: Optional[str] = None
    parameters: Optional[Dict[str, Any]] = None
    
    def __post_init__(self):
        if self.name is None:
            self.name = f"{self.sensor_type}_{uuid4().hex[:8]}"
        if self.parameters is None:
            self.parameters = {}


@dataclass
class Actuator:
    """An actuator (motor, effector) available on the robot"""
    actuator_type: str  # 'motor', 'gripper', 'arm_joint', etc.
    joint_name: str  # name of the joint this actuator controls
    max_force: float  # Newtons
    max_velocity: float  # radians/second or meters/second
    name: Optional[str] = None
    
    def __post_init__(self):
        if self.name is None:
            self.name = f"{self.actuator_type}_{uuid4().hex[:8]}"


@dataclass
class Position3D:
    """3D position in space"""
    x: float
    y: float
    z: float


@dataclass
class Orientation3D:
    """3D orientation as quaternion"""
    x: float
    y: float
    z: float
    w: float


@dataclass
class HumanoidRobot:
    """
    Virtual bipedal robot with sensors, actuators and control systems
    """
    model_name: str
    bipedal_config: BipedalConfig
    robot_id: str = None
    sensors: List[Sensor] = None
    actuators: List[Actuator] = None
    current_position: Position3D = None
    current_orientation: Orientation3D = None
    status: str = "idle"  # 'idle', 'moving', 'executing_task', 'charging', etc.
    battery_level: float = 100.0  # percentage
    created_at: datetime = None
    updated_at: datetime = None
    
    def __post_init__(self):
        if self.robot_id is None:
            self.robot_id = f"robot_{uuid4().hex}"
        if self.sensors is None:
            self.sensors = []
        if self.actuators is None:
            self.actuators = []
        if self.current_position is None:
            self.current_position = Position3D(0.0, 0.0, 0.0)
        if self.current_orientation is None:
            self.current_orientation = Orientation3D(0.0, 0.0, 0.0, 1.0)  # No rotation
        if self.created_at is None:
            self.created_at = datetime.now()
        self.updated_at = datetime.now()
    
    def add_sensor(self, sensor: Sensor) -> None:
        """Add a sensor to the robot"""
        self.sensors.append(sensor)
        self.updated_at = datetime.now()
    
    def remove_sensor(self, sensor_name: str) -> bool:
        """Remove a sensor from the robot by name"""
        for i, sensor in enumerate(self.sensors):
            if sensor.name == sensor_name:
                del self.sensors[i]
                self.updated_at = datetime.now()
                return True
        return False
    
    def add_actuator(self, actuator: Actuator) -> None:
        """Add an actuator to the robot"""
        self.actuators.append(actuator)
        self.updated_at = datetime.now()
    
    def remove_actuator(self, actuator_name: str) -> bool:
        """Remove an actuator from the robot by name"""
        for i, actuator in enumerate(self.actuators):
            if actuator.name == actuator_name:
                del self.actuators[i]
                self.updated_at = datetime.now()
                return True
        return False
    
    def update_position(self, new_position: Position3D, new_orientation: Orientation3D) -> None:
        """Update the robot's position and orientation"""
        self.current_position = new_position
        self.current_orientation = new_orientation
        self.updated_at = datetime.now()
    
    def update_status(self, new_status: str) -> None:
        """Update the robot's operational status"""
        self.status = new_status
        self.updated_at = datetime.now()
    
    def update_battery_level(self, new_level: float) -> None:
        """Update the robot's battery level"""
        if 0.0 <= new_level <= 100.0:
            self.battery_level = new_level
            self.updated_at = datetime.now()
    
    def validate(self) -> bool:
        """Validate that the robot has valid sensor and actuator configurations"""
        # Check bipedal configuration
        if not isinstance(self.bipedal_config, BipedalConfig):
            return False
        if self.bipedal_config.leg_length <= 0:
            return False
        if self.bipedal_config.hip_width <= 0:
            return False
        if self.bipedal_config.max_step_size <= 0:
            return False
        
        # Check sensors
        for sensor in self.sensors:
            if not isinstance(sensor, Sensor):
                return False
        
        # Check actuators
        for actuator in self.actuators:
            if not isinstance(actuator, Actuator):
                return False
            if actuator.max_force <= 0:
                return False
            if actuator.max_velocity <= 0:
                return False
        
        # Check position and orientation
        if not isinstance(self.current_position, Position3D):
            return False
        if not isinstance(self.current_orientation, Orientation3D):
            return False
        
        return True