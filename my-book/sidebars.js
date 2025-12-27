// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: Introduction to AI-Robotics',
      items: [
        'module-1/index',
        'module-1/ros2-nervous-system',
        'module-1/python-agents-urdf',
        'module-1/nodes-topics-services',
        'module-1/glossary',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation & Physics',
      items: [
        'module-2/unity-rendering',
        'module-2/gazebo-physics',
        'module-2/sensors',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Simulation & Perception',
      items: [
        'module-3-simulation/intro',
        'module-3-simulation/isaac-sim-synthetic-data',
        'module-3-simulation/perception-systems',
        'module-3-simulation/path-planning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Planning & Integration',
      items: [
        'module-4-integration/intro',
        'module-4-integration/voice-to-action',
        'module-4-integration/cognitive-planning',
        'module-4-integration/capstone-autonomous-humanoid',
      ],
    },
  ],
};

module.exports = sidebars;