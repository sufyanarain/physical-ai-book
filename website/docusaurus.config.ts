import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// For Netlify deployment, update 'url' to your Netlify domain
const config: Config = {
  title: '🤖 Physical AI & Humanoid Robotics',
  tagline: 'Master the future of embodied intelligence',
  favicon: 'img/favicon.ico',

  // Update this to your Netlify domain after deployment (e.g., https://your-site-name.netlify.app)
  url: process.env.URL || 'https://physical-ai-book.netlify.app',
  baseUrl: '/',

  organizationName: 'sufyanarain',
  projectName: 'physical-ai-book',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl:
            'https://github.com/yourusername/physical-ai-textbook/tree/main/website/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'docs-software',
        path: 'docs-software',
        routeBasePath: 'docs-software',
        sidebarPath: './sidebars.ts',
        editUrl: 'https://github.com/sufyanarain/physical-ai-book/tree/main/website/',
      },
    ],
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'docs-hardware',
        path: 'docs-hardware',
        routeBasePath: 'docs-hardware',
        sidebarPath: './sidebars.ts',
        editUrl: 'https://github.com/sufyanarain/physical-ai-book/tree/main/website/',
      },
    ],
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'docs-urdu',
        path: 'docs-urdu',
        routeBasePath: 'docs-urdu',
        sidebarPath: './sidebars-urdu.ts',
        editUrl: 'https://github.com/sufyanarain/physical-ai-book/tree/main/website/',
      },
    ],
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'docs-urdu-software',
        path: 'docs-urdu-software',
        routeBasePath: 'docs-urdu-software',
        sidebarPath: './sidebars-urdu.ts',
        editUrl: 'https://github.com/sufyanarain/physical-ai-book/tree/main/website/',
      },
    ],
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'docs-urdu-hardware',
        path: 'docs-urdu-hardware',
        routeBasePath: 'docs-urdu-hardware',
        sidebarPath: './sidebars-urdu.ts',
        editUrl: 'https://github.com/sufyanarain/physical-ai-book/tree/main/website/',
      },
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    navbar: {
      title: 'Physical AI',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {
          href: 'https://github.com/yourusername/physical-ai-textbook',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Textbook',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'Get Started',
              to: '/docs/module-1/ros2-intro',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/yourusername/physical-ai-textbook',
            },
            {
              label: 'Panaversity',
              href: 'https://panaversity.org',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Textbook. Created by <strong>Muhammad Sufyan</strong> for Panaversity Hackathon.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'json', 'yaml', 'cpp'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
