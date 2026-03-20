// @ts-check
import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';

// https://astro.build/config
export default defineConfig({
	site: 'https://dujeong.github.io',
	base: '/antbot_wiki',
	integrations: [
		starlight({
			title: 'AntBot Wiki',
			favicon: '/favicon.png',
			defaultLocale: 'root',
			locales: {
				root: { label: '한국어', lang: 'ko' },
				en: { label: 'English', lang: 'en' },
			},
			social: [{ icon: 'github', label: 'GitHub', href: 'https://github.com/ROBOTIS-move/antbot' }],
			customCss: ['./src/styles/custom.css'],
			sidebar: [
				{
					label: 'Introduction',
					translations: { ko: '소개' },
					autogenerate: { directory: 'introduction' },
				},
				{
					label: 'Hardware',
					translations: { ko: '하드웨어' },
					autogenerate: { directory: 'hardware' },
				},
				{
					label: 'Software',
					translations: { ko: '소프트웨어' },
					autogenerate: { directory: 'software' },
				},
				{
					label: 'Quick Start',
					translations: { ko: '빠른 시작' },
					autogenerate: { directory: 'quick-start' },
				},
				{
					label: 'Development Guide',
					translations: { ko: '소프트웨어 개발 가이드' },
					autogenerate: { directory: 'development-guide' },
				},
				{
					label: 'Maintenance & Troubleshooting',
					translations: { ko: '유지보수 및 트러블슈팅' },
					autogenerate: { directory: 'maintenance' },
				},
				{
					label: 'License & Support',
					translations: { ko: '라이선스 및 기술 지원' },
					autogenerate: { directory: 'license' },
				},
			],
		}),
	],
});
