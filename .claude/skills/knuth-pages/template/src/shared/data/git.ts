import 'server-only'

import { execFile } from 'node:child_process'
import { promisify } from 'node:util'
import type { Commit } from '@/shared/lib/types'
import { repoRoot } from './repo-root'

const run = promisify(execFile)

// Unit separator between fields, record separator between commits — commit
// subjects can contain anything short of control characters.
const FORMAT = '%H%x1f%an%x1f%aI%x1f%s%x1e'

/**
 * Latest commits of the host repo via `git log --format`, newest first.
 * A directory that is not a git repo (or a host without git) yields [].
 */
export async function readRecentCommits(limit = 15): Promise<Commit[]> {
  let stdout: string
  try {
    ;({ stdout } = await run(
      'git',
      ['log', '-n', String(limit), `--format=${FORMAT}`],
      { cwd: repoRoot },
    ))
  } catch {
    return []
  }
  return stdout
    .split('\x1e')
    .map((record) => record.trim())
    .filter(Boolean)
    .map((record) => {
      const [hash = '', author = '', date = '', subject = ''] = record.split('\x1f')
      return { hash, author, date, subject }
    })
}
