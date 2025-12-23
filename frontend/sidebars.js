/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Foundations',
      link: {
        type: 'doc',
        id: 'modules/module-1-ros2/index',
      },
      items: [
        'modules/module-1-ros2/chapter-01-intro',
        'modules/module-1-ros2/chapter-02-comms',
        'modules/module-1-ros2/chapter-03-urdf',
        'modules/module-1-ros2/chapter-04-ai-bridge',
        'modules/module-1-ros2/chapter-05-sensorimotor-learning',
        {
          type: 'category',
          label: 'Sensorimotor Learning Resources',
          items: [
            'modules/module-1-ros2/exercises-sensorimotor',
            'modules/module-1-ros2/diagrams-sensorimotor',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      link: {
        type: 'doc',
        id: 'modules/module-2-digital-twin/index',
      },
      items: [
        'modules/module-2-digital-twin/chapter-05-intro',
        'modules/module-2-digital-twin/chapter-06-gazebo',
        'modules/module-2-digital-twin/chapter-07-sensors',
        'modules/module-2-digital-twin/chapter-08-unity',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac',
      link: {
        type: 'doc',
        id: 'modules/module-3-isaac/index',
      },
      items: [
        'modules/module-3-isaac/chapter-09-platform',
        'modules/module-3-isaac/chapter-10-sim',
        'modules/module-3-isaac/chapter-11-perception',
        'modules/module-3-isaac/chapter-12-nav2',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA Systems',
      link: {
        type: 'doc',
        id: 'modules/module-4-vla/index',
      },
      items: [
        'modules/module-4-vla/chapter-13-vla-intro',
        'modules/module-4-vla/chapter-14-speech',
        'modules/module-4-vla/chapter-15-llm-planning',
        'modules/module-4-vla/chapter-16-conversation',
      ],
    },
    {
      type: 'doc',
      id: 'capstone/index',
      label: 'Capstone Project',
    },
  ],
};

export default sidebars;
