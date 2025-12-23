// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros2/1-ros2-foundations',
        'module-1-ros2/2-python-agents-ros-control',
        'module-1-ros2/3-humanoid-modeling-urdf'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-digital-twin/1-physics-simulation-gazebo',
        'module-2-digital-twin/2-high-fidelity-interaction-unity',
        'module-2-digital-twin/3-simulated-sensors'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3-isaac/1-nvidia-isaac-sim-synthetic-data',
        'module-3-isaac/2-isaac-ros-accelerated-perception',
        'module-3-isaac/3-nav2-humanoid-path-planning'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) Capstone',
      items: [
        'module-4-vla/1-voice-to-action-speech-recognition',
        'module-4-vla/2-llm-based-cognitive-planning',
        'module-4-vla/3-capstone-autonomous-humanoid'
      ],
    },
  ],
};

export default sidebars;
