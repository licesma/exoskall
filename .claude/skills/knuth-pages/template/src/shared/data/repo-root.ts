import 'server-only'

import { basename, resolve } from 'node:path'

// The site always lives at <repo>/.knuth/pages/ and `next dev` runs with the
// site directory as cwd, so the host repo root is exactly two levels up.
// Derived once, here — every reader imports this constant.
export const repoRoot = resolve(process.cwd(), '..', '..')

export const repoName = basename(repoRoot)
