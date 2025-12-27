"""
Unit Tests for AI-Robot Brain & Vision-Language-Action Implementation
"""
import unittest
from my_book.models.simulation_environment import SimulationEnvironment, PhysicsProperties, LightingConfig
from my_book.models.humanoid_robot import HumanoidRobot, BipedalConfig, Position3D, Orientation3D
from my_book.models.navigation_path import NavigationPath, Waypoint
from my_book.models.perception_data import PerceptionData, EnvironmentObject, Obstacle, EnvironmentState
from my_book.models.voice_command import VoiceCommand
from my_book.models.action_sequence import ActionSequence, ActionStep
from my_book.models.cognitive_plan import CognitivePlan, PlanStep


class TestSimulationEnvironment(unittest.TestCase):
    def test_create_simulation_environment(self):
        """Test creating a simulation environment"""
        physics = PhysicsProperties(gravity=9.81, friction=0.5)
        lighting = LightingConfig()
        
        env = SimulationEnvironment(
            name="Test Environment",
            description="A test environment",
            physics_properties=physics,
            lighting_config=lighting
        )
        
        self.assertEqual(env.name, "Test Environment")
        self.assertTrue(env.validate())
    
    def test_add_remove_objects(self):
        """Test adding and removing objects from environment"""
        physics = PhysicsProperties()
        lighting = LightingConfig()
        
        env = SimulationEnvironment(
            name="Test Environment",
            description="A test environment",
            physics_properties=physics,
            lighting_config=lighting
        )
        
        # Add an object
        from my_book.models.simulation_environment import SimulationObject
        obj = SimulationObject(
            object_type="cube",
            position={"x": 0.0, "y": 0.0, "z": 0.5},
            rotation={"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        )
        env.add_object(obj)
        
        self.assertEqual(len(env.objects), 1)
        
        # Remove the object
        result = env.remove_object(obj.name)
        self.assertTrue(result)
        self.assertEqual(len(env.objects), 0)


class TestHumanoidRobot(unittest.TestCase):
    def test_create_humanoid_robot(self):
        """Test creating a humanoid robot"""
        bipedal_config = BipedalConfig(
            leg_length=0.8,
            hip_width=0.3,
            max_step_size=0.5,
            max_angular_velocity=1.0
        )
        
        robot = HumanoidRobot(
            model_name="Test Robot",
            bipedal_config=bipedal_config
        )
        
        self.assertEqual(robot.model_name, "Test Robot")
        self.assertTrue(robot.validate())
    
    def test_update_position(self):
        """Test updating robot position"""
        bipedal_config = BipedalConfig(
            leg_length=0.8,
            hip_width=0.3,
            max_step_size=0.5,
            max_angular_velocity=1.0
        )
        
        robot = HumanoidRobot(
            model_name="Test Robot",
            bipedal_config=bipedal_config
        )
        
        new_position = Position3D(1.0, 2.0, 0.0)
        new_orientation = Orientation3D(0.0, 0.0, 0.0, 1.0)
        
        robot.update_position(new_position, new_orientation)
        
        self.assertEqual(robot.current_position.x, 1.0)
        self.assertEqual(robot.current_position.y, 2.0)


class TestNavigationPath(unittest.TestCase):
    def test_create_navigation_path(self):
        """Test creating a navigation path"""
        path = NavigationPath(
            start_position={"x": 0.0, "y": 0.0, "z": 0.0},
            end_position={"x": 5.0, "y": 3.0, "z": 0.0}
        )
        
        self.assertEqual(path.start_position["x"], 0.0)
        self.assertEqual(path.end_position["y"], 3.0)
        self.assertTrue(path.validate())
    
    def test_add_waypoint(self):
        """Test adding a waypoint to the path"""
        path = NavigationPath(
            start_position={"x": 0.0, "y": 0.0, "z": 0.0},
            end_position={"x": 5.0, "y": 3.0, "z": 0.0}
        )
        
        waypoint = Waypoint(
            position={"x": 2.0, "y": 1.0, "z": 0.0},
            description="First waypoint"
        )
        
        path.add_waypoint(waypoint)
        
        self.assertEqual(len(path.waypoints), 1)
        self.assertEqual(path.waypoints[0].description, "First waypoint")


class TestPerceptionData(unittest.TestCase):
    def test_create_perception_data(self):
        """Test creating perception data"""
        perception_data = PerceptionData(
            robot_id="test_robot_123",
            sensor_type="camera"
        )
        
        self.assertEqual(perception_data.robot_id, "test_robot_123")
        self.assertEqual(perception_data.sensor_type, "camera")
        self.assertTrue(perception_data.validate())
    
    def test_add_camera_data(self):
        """Test adding camera data to perception data"""
        perception_data = PerceptionData(
            robot_id="test_robot_123",
            sensor_type="camera"
        )
        
        # Create mock camera data
        from my_book.models.perception_data import CameraData
        camera_data = CameraData(
            image_data="base64_encoded_image",
            resolution={"width": 640, "height": 480}
        )
        
        perception_data.add_camera_data(camera_data)
        
        self.assertIn("camera", perception_data.raw_data)


class TestVoiceCommand(unittest.TestCase):
    def test_create_voice_command(self):
        """Test creating a voice command"""
        command = VoiceCommand(
            original_text="Move forward by 1 meter"
        )
        
        self.assertEqual(command.original_text, "Move forward by 1 meter")
        self.assertTrue(command.validate())
    
    def test_update_intent(self):
        """Test updating the intent of a voice command"""
        command = VoiceCommand(
            original_text="Move forward by 1 meter"
        )
        
        command.update_intent("move_robot", 0.95)
        
        self.assertEqual(command.intent, "move_robot")
        self.assertEqual(command.confidence_score, 0.95)


class TestActionSequence(unittest.TestCase):
    def test_create_action_sequence(self):
        """Test creating an action sequence"""
        sequence = ActionSequence(
            name="Test Action Sequence"
        )
        
        self.assertEqual(sequence.name, "Test Action Sequence")
        self.assertTrue(sequence.validate())
    
    def test_add_action_step(self):
        """Test adding an action step to a sequence"""
        sequence = ActionSequence(
            name="Test Action Sequence"
        )
        
        step = ActionStep(
            action_type="move_base",
            parameters={"direction": "forward", "distance": 1.0},
            description="Move forward by 1 meter"
        )
        
        sequence.add_step(step)
        
        self.assertEqual(len(sequence.steps), 1)
        self.assertEqual(sequence.steps[0].action_type, "move_base")


class TestCognitivePlan(unittest.TestCase):
    def test_create_cognitive_plan(self):
        """Test creating a cognitive plan"""
        plan = CognitivePlan(
            input_command="Go to kitchen and pick up red cup"
        )
        
        self.assertEqual(plan.input_command, "Go to kitchen and pick up red cup")
        self.assertTrue(plan.validate())
    
    def test_add_plan_step(self):
        """Test adding a step to a cognitive plan"""
        plan = CognitivePlan(
            input_command="Go to kitchen and pick up red cup"
        )
        
        step = PlanStep(
            description="Navigate to kitchen",
        )
        
        plan.add_step(step)
        
        self.assertEqual(len(plan.breakdown), 1)
        self.assertEqual(plan.breakdown[0].description, "Navigate to kitchen")


if __name__ == '__main__':
    unittest.main()