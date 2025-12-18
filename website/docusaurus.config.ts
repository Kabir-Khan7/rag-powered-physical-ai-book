import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI Textbook',
  tagline: 'Hands-on guide for humanoid robotics and embodied intelligence',
  favicon: 'img/favicon.ico',
  future: {
    v4: true,
  },
  url: 'https://physical-ai-textbook.vercel.app',
  baseUrl: '/',
  organizationName: 'physical-ai-lab',
  projectName: 'hackathon-textbook',
  onBrokenLinks: 'throw',
  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },
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
          routeBasePath: '/',
          showLastUpdateTime: false,
          editUrl: undefined,
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
      '@easyops-cn/docusaurus-search-local',
      {
        hashed: true,
        indexDocs: true,
        docsRouteBasePath: '/',
        searchResultLimits: 10,
        highlightSearchTermsOnTargetPage: true,
      },
    ],
  ],
  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
      defaultMode: 'light',
    },
    navbar: {
      title: 'Physical AI Textbook',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Chapters',
        },
        {
          href: 'https://github.com/physical-ai-lab/hackathon-textbook',
          label: 'Repository',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Product',
          items: [
            {label: 'Read the Textbook', to: '/'},
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Discussions',
              href: 'https://github.com/physical-ai-lab/hackathon-textbook/discussions',
            },
            {
              label: 'Support',
              href: 'mailto:support@physicalai.school',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {label: 'Repository', href: 'https://github.com/physical-ai-lab/hackathon-textbook'},
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
