import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'ماڈیول 1: روبوٹک اعصابی نظام (ROS 2)',
      items: [
        'module-1/ros2-intro',
        'module-1/ros2-fundamentals',
      ],
    },
    {
      type: 'category',
      label: 'ماڈیول 2: ڈیجیٹل ٹون (Gazebo & Unity)',
      items: [
        'module-2/simulation-intro',
        'module-2/gazebo-unity',
      ],
    },
    {
      type: 'category',
      label: 'ماڈیول 3: اے آئی-روبوٹ برین (NVIDIA Isaac)',
      items: [
        'module-3/isaac-intro',
        'module-3/isaac-advanced',
      ],
    },
    {
      type: 'category',
      label: 'ماڈیول 4: ویژن-لینگویج-ایکشن (VLA)',
      items: [
        'module-4/vla-intro',
        'module-4/capstone-project',
      ],
    },
  ],
};

export default sidebars;
