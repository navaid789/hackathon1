// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Part I: Foundations',
      collapsible: true,
      collapsed: false,
      items: [
        'chapter-1-introduction-to-physical-ai/index',
        'chapter-2-sensors-and-perception-in-physical-world/index',
        'chapter-3-the-robotic-nervous-system-ros-2/index',
        'chapter-4-robot-body-representation-urdf-kinematics/index'
      ],
    },
    {
      type: 'category',
      label: 'Part II: Simulation & Middleware',
      collapsible: true,
      collapsed: false,
      items: [
        'chapter-5-digital-twins-physics-simulation/index',
        'chapter-6-nvidia-isaac-sim-synthetic-data/index'
      ],
    },
    {
      type: 'category',
      label: 'Part III: Perception & Action',
      collapsible: true,
      collapsed: false,
      items: [
        'chapter-7-ai-perception-and-navigation/index',
        'chapter-8-learning-to-act-reinforcement-learning/index',
        'chapter-9-sim-to-real-transfer/index'
      ],
    },
    {
      type: 'category',
      label: 'Part IV: Advanced Systems',
      collapsible: true,
      collapsed: false,
      items: [
        'chapter-10-vision-language-action-vla-systems/index',
        'chapter-11-conversational-robotics/index',
        'chapter-12-capstone-the-autonomous-humanoid/index',
        'chapter-13-hardware-architecture-lab-design/index',
        'chapter-14-future-of-physical-ai/index'
      ],
    },
  ],
};

module.exports = sidebars;