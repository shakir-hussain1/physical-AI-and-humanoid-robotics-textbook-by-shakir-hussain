// @ts-check
import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive textbook for building intelligent humanoid robots',
  favicon: 'img/favicon.ico',

  url: 'https://shakir-hussain1.github.io',
  baseUrl: '/physical-AI-and-humanoid-robotics-textbook-by-shakir-hussain/',

  organizationName: 'shakir-hussain1',
  projectName: 'physical-AI-and-humanoid-robotics-textbook-by-shakir-hussain',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  markdown: {
    mermaid: true,
  },

  themes: ['@docusaurus/theme-mermaid'],

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          editUrl:
            'https://github.com/shakir-hussain1/physical-AI-and-humanoid-robotics-textbook-by-shakir-hussain/tree/master/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Robotics Logo',
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
            href: 'https://github.com/shakir-hussain1/physical-AI-and-humanoid-robotics-textbook-by-shakir-hussain',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Modules',
            items: [
              {
                label: 'Module 1: ROS 2 Foundations',
                to: '/docs/modules/module-1-ros2',
              },
              {
                label: 'Module 2: Digital Twin',
                to: '/docs/modules/module-2-digital-twin',
              },
              {
                label: 'Module 3: NVIDIA Isaac',
                to: '/docs/modules/module-3-isaac',
              },
              {
                label: 'Module 4: VLA Systems',
                to: '/docs/modules/module-4-vla',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'Code Examples',
                href: 'https://github.com/shakir-hussain1/physical-AI-and-humanoid-robotics-textbook-by-shakir-hussain/tree/master/docs/code-examples',
              },
              {
                label: 'GitHub',
                href: 'https://github.com/shakir-hussain1/physical-AI-and-humanoid-robotics-textbook-by-shakir-hussain',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['python', 'bash', 'yaml', 'json', 'markup'],
      },
      mermaid: {
        theme: {light: 'neutral', dark: 'dark'},
      },
    }),
};

export default config;
