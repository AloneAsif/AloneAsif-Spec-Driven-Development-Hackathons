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
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      items: ['module-1/intro-to-ros2','module-1/ros2-pub-sub','module-1/ros2-python-agents','module-1/urdf-basics'],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      items: ['module-2/building-digital-twin','module-2/gazebo-simulation','module-2/unity-rendering','module-2/troubleshooting','module-2/validation'],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: ['module-3/index', 'module-3/isaac-sim-overview', 'module-3/isaac-ros-perception', 'module-3/nav2-humanoid-navigation', 'module-3/system-integration', 'module-3/module-summary'],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/intro',
        'module-4/vla-architecture',
        'module-4/voice-to-text-whisper',
        'module-4/llm-task-planning',
        'module-4/perception-feedback-loop',
        'module-4/ros2-action-execution',
        'module-4/capstone-autonomous-humanoid',
        'module-4/evaluation-criteria'
      ],
    },
  ],
};

export default sidebars;


