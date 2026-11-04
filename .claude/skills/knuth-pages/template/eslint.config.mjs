// eslint.config.mjs
import { defineConfig, globalIgnores } from 'eslint/config'
import nextVitals from 'eslint-config-next/core-web-vitals'
import nextTs from 'eslint-config-next/typescript'
import boundaries from 'eslint-plugin-boundaries'
import checkFile from 'eslint-plugin-check-file'
import importX from 'eslint-plugin-import-x'
import { createTypeScriptImportResolver } from 'eslint-import-resolver-typescript'

const NO_PARENT_IMPORTS = {
  group: ['../**'],
  message: 'No ../ imports. Same folder: "./x". Anywhere else: "@/...".',
}

export default defineConfig([
  ...nextVitals,
  ...nextTs,
  globalIgnores(['.next/**', 'out/**', 'build/**', 'next-env.d.ts']),

  // ---- Architecture: closed-world layer map -------------------------------
  {
    files: ['src/**/*.{ts,tsx}'],
    plugins: { boundaries },
    settings: {
      'import/resolver': { typescript: { alwaysTryTypes: true } },
      'boundaries/elements': [
        { type: 'shared-ui', pattern: 'src/shared/ui' },
        { type: 'shared-lib', pattern: 'src/shared/lib' },
        { type: 'shared-data', pattern: 'src/shared/data' },
        { type: 'widget', pattern: 'src/widgets/*', capture: ['widget'] },
        { type: 'route', pattern: 'src/app/*', capture: ['route'] },
        { type: 'app-core', pattern: 'src/app' },
      ],
    },
    rules: {
      'boundaries/no-unknown-files': 2,
      'boundaries/no-unknown-dependencies': 2,
      'boundaries/dependencies': [2, {
        default: 'disallow',
        policies: [
          { from: { element: { type: 'route' } },
            allow: { to: { element: { types: { anyOf: ['widget', 'shared-ui', 'shared-lib', 'shared-data'] } } } } },
          { from: { element: { type: 'app-core' } },
            allow: { to: { element: { types: { anyOf: ['app-core', 'widget', 'shared-ui', 'shared-lib', 'shared-data'] } } } } },
          { from: { element: { type: 'widget' } },
            allow: { to: { element: { types: { anyOf: ['shared-ui', 'shared-lib'] } } } } },
          { from: { element: { type: 'shared-ui' } },
            allow: { to: { element: { types: { anyOf: ['shared-ui', 'shared-lib'] } } } } },
          { from: { element: { type: 'shared-lib' } },
            allow: { to: { element: { type: 'shared-lib' } } } },
          { from: { element: { type: 'shared-data' } },
            allow: { to: { element: { types: { anyOf: ['shared-data', 'shared-lib'] } } } } },
        ],
      }],
    },
  },

  // ---- I/O quarantine: reads only inside src/shared/data ------------------
  {
    files: ['src/**/*.{ts,tsx}'],
    ignores: ['src/shared/data/**'],
    rules: {
      'no-restricted-globals': ['error',
        { name: 'fetch', message: 'All reads live in src/shared/data. Pages call readers; clients get props.' }],
      'no-restricted-properties': ['error',
        { object: 'globalThis', property: 'fetch' },
        { object: 'window', property: 'fetch' }],
      'no-restricted-imports': ['error', {
        patterns: [
          NO_PARENT_IMPORTS,
          {
            group: ['node:*', 'fs', 'fs/*', 'path', 'os', 'child_process', 'better-sqlite3', 'server-only'],
            message: 'Node/system access only inside src/shared/data.',
          },
        ],
      }],
    },
  },
  {
    // flat-config rule entries REPLACE (not merge) earlier ones per file, so
    // shared/data needs its own no-restricted-imports carrying the ../ ban.
    files: ['src/shared/data/**/*.{ts,tsx}'],
    rules: {
      'no-restricted-imports': ['error', { patterns: [NO_PARENT_IMPORTS] }],
    },
  },

  // ---- Shape: one component per file, no barrels, naming, cycles ----------
  {
    files: ['src/**/*.{ts,tsx}'],
    plugins: { 'check-file': checkFile, 'import-x': importX },
    settings: {
      'import-x/resolver-next': [
        createTypeScriptImportResolver({ alwaysTryTypes: true }),
      ],
      // without this, no-cycle silently skips .ts/.tsx targets (found by testing)
      'import-x/parsers': { '@typescript-eslint/parser': ['.ts', '.tsx'] },
      'import-x/extensions': ['.js', '.jsx', '.ts', '.tsx'],
    },
    rules: {
      'react/no-multi-comp': 'error',
      'import-x/no-cycle': ['error', { maxDepth: 4 }],
      'import-x/no-self-import': 'error',
      'no-restricted-syntax': ['error', {
        selector: 'ExportAllDeclaration',
        message: 'No barrels. Import the concrete file.',
      }],
      'check-file/filename-blocklist': ['error', { 'src/**/index.{ts,tsx}': '*.tsx' }],
      'check-file/filename-naming-convention': ['error',
        { 'src/**/*.{ts,tsx}': 'KEBAB_CASE' }, { ignoreMiddleExtensions: true }],
      'check-file/folder-naming-convention': ['error', {
        'src/app/**/': 'NEXT_JS_APP_ROUTER_CASE',
        'src/widgets/**/': 'KEBAB_CASE',
        'src/shared/**/': 'KEBAB_CASE',
      }],
    },
  },

  // shadcn kit files ship component families; exempt from one-per-file
  {
    files: ['src/shared/ui/**'],
    rules: { 'react/no-multi-comp': 'off' },
  },
])
