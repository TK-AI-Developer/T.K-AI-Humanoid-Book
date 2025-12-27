"""
Navigation API Endpoints
Implements API endpoints for navigation functionality
"""
from typing import Dict, Any
from flask import Flask, request, jsonify
from ..services.navigation_service import NavigationService
import uuid


class NavigationAPI:
    """
    API endpoints for navigation functionality
    """
    
    def __init__(self, app: Flask, navigation_service: NavigationService):
        self.app = app
        self.navigation_service = navigation_service
        self.register_routes()
    
    def register_routes(self):
        """Register the navigation API routes"""
        
        @self.app.route('/api/robots/<robot_id>/navigation/plan', methods=['POST'])
        def plan_path(robot_id: str):
            """
            Plan a navigation path for the robot
            """
            try:
                data = request.get_json()
                
                # Validate required fields
                if 'destination' not in data:
                    return jsonify({"error": "Missing required field: destination"}), 400
                
                path_id = self.navigation_service.plan_path(
                    robot_id=robot_id,
                    destination=data['destination'],
                    path_type=data.get('path_type', 'optimal'),
                    avoid_dynamic_obstacles=data.get('avoid_dynamic_obstacles', True)
                )
                
                # Get the path to return its details
                path = self.navigation_service.navigation_paths[path_id]
                
                return jsonify({
                    "path_id": path_id,
                    "waypoints": [
                        {
                            "position": wp.position,
                            "description": wp.description
                        } for wp in path.waypoints
                    ],
                    "estimated_duration": path.estimated_duration,
                    "distance": path.distance
                }), 200
                
            except Exception as e:
                return jsonify({"error": str(e)}), 500
        
        @self.app.route('/api/robots/<robot_id>/navigation/execute', methods=['POST'])
        def execute_navigation(robot_id: str):
            """
            Execute a planned navigation path
            """
            try:
                data = request.get_json()
                
                # Validate required fields
                if 'path_id' not in data:
                    return jsonify({"error": "Missing required field: path_id"}), 400
                
                nav_id = self.navigation_service.execute_navigation(
                    robot_id=robot_id,
                    path_id=data['path_id'],
                    speed=data.get('speed', 1.0)
                )
                
                return jsonify({
                    "status": "executing",
                    "estimated_completion": "2025-12-25T10:05:00Z"  # In a real implementation, this would be calculated
                }), 200
                
            except Exception as e:
                return jsonify({"error": str(e)}), 500


class PerceptionAPI:
    """
    API endpoints for perception functionality
    """
    
    def __init__(self, app: Flask, navigation_service: NavigationService):
        self.app = app
        self.navigation_service = navigation_service
        self.register_routes()
    
    def register_routes(self):
        """Register the perception API routes"""
        
        @self.app.route('/api/robots/<robot_id>/perception', methods=['GET'])
        def get_perception_data(robot_id: str):
            """
            Retrieve perception data from robot sensors
            """
            try:
                perception_result = self.navigation_service.get_perception_data(robot_id)
                
                return jsonify(perception_result), 200
                
            except Exception as e:
                return jsonify({"error": str(e)}), 500


# Example usage
if __name__ == "__main__":
    app = Flask(__name__)
    navigation_service = NavigationService()
    navigation_api = NavigationAPI(app, navigation_service)
    perception_api = PerceptionAPI(app, navigation_service)
    
    # This would be run with app.run() in a real implementation
    print("Navigation and Perception API endpoints registered")