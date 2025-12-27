# AI-Robot Brain & Vision-Language-Action (VLA) for Humanoid Robotics

This repository contains educational modules for AI-Robot Brain & Vision-Language-Action (VLA) for humanoid robotics, focusing on simulation, perception, navigation, and voice-driven autonomous actions. The implementation includes Docusaurus chapters covering simulation environments, perception systems, path planning, and voice-to-action integration.

## Modules

### Module 3: Simulation & Perception
1. [Isaac Sim & Synthetic Data](./docs/module-3-simulation/isaac-sim-synthetic-data.md) - Setting up photorealistic simulation environments
2. [Perception Systems](./docs/module-3-simulation/perception-systems.md) - VSLAM, sensor fusion, navigation pipeline
3. [Path Planning](./docs/module-3-simulation/path-planning.md) - Bipedal movement, obstacle avoidance, trajectory planning

### Module 4: Planning & Integration
1. [Voice-to-Action](./docs/module-4-integration/voice-to-action.md) - Converting voice commands to executable actions
2. [Cognitive Planning](./docs/module-4-integration/cognitive-planning.md) - Mapping natural language to action sequences
3. [Capstone Autonomous Humanoid](./docs/module-4-integration/capstone-autonomous-humanoid.md) - Integrating perception, planning, navigation, and manipulation

## Architecture

The implementation follows a service-oriented architecture with the following components:

- **Models**: Data structures for simulation environments, robots, navigation paths, perception data, voice commands, action sequences, and cognitive plans
- **Services**: Business logic for simulation, navigation, voice processing, cognitive planning, and integration
- **APIs**: Endpoints for interacting with the system
- **Scenarios**: Simulation scenarios for testing and training

## Setup

1. Install Python 3.11
2. Install dependencies as specified in your project requirements
3. Set up Docusaurus for documentation
4. Configure Isaac Sim and ROS 2 for simulation environments

## Running the Examples

Documentation and code examples are structured to guide students through the implementation of each module. Follow the chapters in order for a progressive learning experience.

## Technologies Used

- Python 3.11
- Docusaurus for documentation
- NVIDIA Isaac Sim for simulation
- ROS 2 for robotics framework
- OpenAI Whisper API for voice processing
- LLMs for cognitive planning

## Contributing

This is an educational project. Contributions that improve clarity, add examples, or enhance the learning experience are welcome.

## License

This project is educational in nature and intended for learning purposes.